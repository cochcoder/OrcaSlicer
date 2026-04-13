#include <catch2/catch_all.hpp>

#include "libslic3r/BinaryGCode.hpp"
#include "libslic3r/Utils.hpp"
#include <boost/filesystem.hpp>
#include <boost/nowide/fstream.hpp>
#include <stdexcept>
#define NANOSVG_IMPLEMENTATION
#include "nanosvg/nanosvg.h"
#define NANOSVGRAST_IMPLEMENTATION
#include "nanosvg/nanosvgrast.h"
namespace {

constexpr char BGCODE_TEMP_TEXT_PATTERN[] = "orcaslicer-bgcode-text-%%%%%%%%.gcode";
constexpr char BGCODE_TEMP_BIN_PATTERN[] = "orcaslicer-bgcode-bin-%%%%%%%%.bgcode";
constexpr char BGCODE_TEMP_V2_PATTERN[] = "orcaslicer-bgcode-v2-%%%%%%%%.bgcode";
constexpr char BGCODE_TEMP_PLAIN_PATTERN[] = "orcaslicer-bgcode-plain-%%%%%%%%.bgcode";

struct TempFileScope
{
    std::vector<boost::filesystem::path> paths;
    ~TempFileScope()
    {
        for (const auto& path : paths)
            boost::filesystem::remove(path);
    }
};

std::string read_file_content(const std::string& path)
{
    boost::nowide::ifstream file(path, std::ios::binary);
    if (!file.is_open())
        throw std::runtime_error("Failed to open file for test read: " + path);
    return std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
}

TEST_CASE("sort_remove_duplicates", "[utils]") {
	std::vector<int> data_src = { 3, 0, 2, 1, 15, 3, 5, 6, 3, 1, 0 };
	std::vector<int> data_dst = { 0, 1, 2, 3, 5, 6, 15 };
	Slic3r::sort_remove_duplicates(data_src);
    REQUIRE(data_src == data_dst);
}

TEST_CASE("string_printf", "[utils]") {
    SECTION("Empty format with empty data should return empty string") {
        std::string outs = Slic3r::string_printf("");
        REQUIRE(outs.empty());
    }
    
    SECTION("String output length should be the same as input") {
        std::string outs = Slic3r::string_printf("1234");
        REQUIRE(outs.size() == 4);
    }
    
    SECTION("String format should be interpreted as with sprintf") {
        std::string outs = Slic3r::string_printf("%d %f %s", 10, 11.4, " This is a string");
        char buffer[1024];
        
        sprintf(buffer, "%d %f %s", 10, 11.4, " This is a string");
        
        REQUIRE(outs.compare(buffer) == 0);
    }
    
    SECTION("String format should survive large input data") {
        std::string input(2048, 'A');
        std::string outs = Slic3r::string_printf("%s", input.c_str());
        REQUIRE(outs.compare(input) == 0);
    }
}

TEST_CASE("gcode_extension_detection", "[utils]") {
    REQUIRE(Slic3r::is_gcode_file("test.gcode"));
    REQUIRE(Slic3r::is_gcode_file("test.bgcode"));
    REQUIRE_FALSE(Slic3r::is_gcode_file("test.3mf"));
    REQUIRE(Slic3r::is_bgcode_file("test.bgcode"));
    REQUIRE_FALSE(Slic3r::is_bgcode_file("test.gcode"));
    REQUIRE(Slic3r::BinaryGCode::is_binary_gcode_path("test.bgcode"));
}

TEST_CASE("bgcode_roundtrip_normalization", "[utils][bgcode]") {
    namespace fs = boost::filesystem;
    const fs::path text_path = fs::temp_directory_path() / fs::unique_path(BGCODE_TEMP_TEXT_PATTERN);
    const fs::path bg_path = fs::temp_directory_path() / fs::unique_path(BGCODE_TEMP_BIN_PATTERN);
    TempFileScope cleanup { { text_path, bg_path } };

    const std::string payload = "G1 X1.000 Y2.000 E0.500\n; generated\nM104 S200\n";
    {
        boost::nowide::ofstream file(text_path.string(), std::ios::binary | std::ios::trunc);
        REQUIRE(file.is_open());
        file << payload;
    }

    Slic3r::BinaryGCode::copy_text_to_bgcode_file(text_path.string(), bg_path.string());
    const auto detected = Slic3r::BinaryGCode::detect_file(bg_path.string());
    REQUIRE(detected.is_binary_gcode);
    REQUIRE(detected.major_version == 1);
    REQUIRE(detected.minor_version == 0);

    const auto normalized = Slic3r::BinaryGCode::normalize_for_gcode_parser(bg_path.string());
    REQUIRE(normalized.uses_temporary_path);
    REQUIRE(normalized.parser_path != normalized.source_path);
    REQUIRE(read_file_content(normalized.parser_path) == payload);
    cleanup.paths.push_back(normalized.parser_path);
}

TEST_CASE("bgcode_version_validation", "[utils][bgcode]") {
    namespace fs = boost::filesystem;
    const fs::path bg_path = fs::temp_directory_path() / fs::unique_path(BGCODE_TEMP_V2_PATTERN);
    TempFileScope cleanup { { bg_path } };

    {
        boost::nowide::ofstream file(bg_path.string(), std::ios::binary | std::ios::trunc);
        REQUIRE(file.is_open());
        file << "BGCODE";
        file.put(static_cast<char>(2)); // unsupported major
        file.put(static_cast<char>(0));
        file << "G1 X0\n";
    }

    try {
        (void)Slic3r::BinaryGCode::normalize_for_gcode_parser(bg_path.string());
        FAIL("Expected normalize_for_gcode_parser to throw on unsupported BGCode version 2.0");
    } catch (const Slic3r::BinaryGCode::Error& err) {
        REQUIRE(err.code() == Slic3r::BinaryGCode::ErrorCode::UnsupportedVersion);
    }

}

TEST_CASE("bgcode_plain_text_passthrough", "[utils][bgcode]") {
    namespace fs = boost::filesystem;
    const fs::path bg_path = fs::temp_directory_path() / fs::unique_path(BGCODE_TEMP_PLAIN_PATTERN);
    TempFileScope cleanup { { bg_path } };

    {
        boost::nowide::ofstream file(bg_path.string(), std::ios::binary | std::ios::trunc);
        REQUIRE(file.is_open());
        file << "G1 X10.0\n";
    }

    const auto normalized = Slic3r::BinaryGCode::normalize_for_gcode_parser(bg_path.string());
    REQUIRE_FALSE(normalized.uses_temporary_path);
    REQUIRE(normalized.parser_path == bg_path.string());
}

}
