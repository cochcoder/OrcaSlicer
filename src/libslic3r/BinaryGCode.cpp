#include "BinaryGCode.hpp"

#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <boost/nowide/cstdio.hpp>
#include <boost/nowide/fstream.hpp>
#include <algorithm>

namespace Slic3r::BinaryGCode {

namespace {

constexpr size_t PROBE_SIZE = 4096;
// Header probe currently expects: "BGCODE" + 2 bytes (major/minor version).
constexpr char BGCODE_MAGIC[] = "BGCODE";
constexpr size_t BGCODE_MAGIC_SIZE = sizeof(BGCODE_MAGIC) - 1;
constexpr size_t BGCODE_MAJOR_VERSION_OFFSET = BGCODE_MAGIC_SIZE;
constexpr size_t BGCODE_MINOR_VERSION_OFFSET = BGCODE_MAGIC_SIZE + 1;
constexpr unsigned char SUPPORTED_MAJOR_VERSION = 1;
constexpr unsigned char SUPPORTED_MINOR_VERSION = 0;

std::string read_probe(const std::string& path)
{
    boost::nowide::ifstream file(path, std::ios::binary);
    if (!file.is_open())
        throw Error(ErrorCode::IoError, "Unable to open file for BGCode detection: " + path);

    std::string probe(PROBE_SIZE, '\0');
    file.read(probe.data(), static_cast<std::streamsize>(probe.size()));
    probe.resize(static_cast<size_t>(file.gcount()));
    return probe;
}

std::string read_all(const std::string& path)
{
    boost::nowide::ifstream file(path, std::ios::binary);
    if (!file.is_open())
        throw Error(ErrorCode::IoError, "Unable to open BGCode file: " + path);
    return std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
}

void write_all(const std::string& path, const std::string& data)
{
    boost::nowide::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file.is_open())
        throw Error(ErrorCode::IoError, "Unable to write BGCode file: " + path);
    file.write(data.data(), static_cast<std::streamsize>(data.size()));
    if (!file.good())
        throw Error(ErrorCode::IoError, "Failed writing BGCode file: " + path);
}

bool contains_nul(const std::string& data)
{
    return data.find('\0') != std::string::npos;
}

} // namespace

Error::Error(ErrorCode code, const std::string& message)
    : std::runtime_error(message)
    , m_code(code)
{
}

bool is_binary_gcode_path(const std::string& path)
{
    return boost::iends_with(path, ".bgcode");
}

DetectionResult detect_file(const std::string& path)
{
    DetectionResult ret;
    const std::string probe = read_probe(path);
    ret.looks_binary_payload = contains_nul(probe);

    const bool has_magic = probe.size() >= BGCODE_MAGIC_SIZE && std::equal(BGCODE_MAGIC, BGCODE_MAGIC + BGCODE_MAGIC_SIZE, probe.begin());
    ret.is_binary_gcode = is_binary_gcode_path(path) || has_magic;

    if (has_magic && probe.size() >= BGCODE_MAGIC_SIZE + 2) {
        ret.major_version = static_cast<unsigned char>(probe[BGCODE_MAJOR_VERSION_OFFSET]);
        ret.minor_version = static_cast<unsigned char>(probe[BGCODE_MINOR_VERSION_OFFSET]);
    }

    return ret;
}

NormalizedInput normalize_for_gcode_parser(const std::string& path)
{
    const DetectionResult detection = detect_file(path);
    if (!detection.is_binary_gcode)
        return { path, path, false };

    const std::string content = read_all(path);
    if (content.empty())
        throw Error(ErrorCode::InvalidData, "The selected BGCode file is empty: " + path);

    const bool has_magic = content.size() >= BGCODE_MAGIC_SIZE && std::equal(BGCODE_MAGIC, BGCODE_MAGIC + BGCODE_MAGIC_SIZE, content.begin());
    if (!has_magic) {
        if (contains_nul(content))
            throw Error(ErrorCode::UnsupportedEncoding, "The selected .bgcode file contains unsupported binary payload.");
        return { path, path, false };
    }

    if (content.size() < BGCODE_MAGIC_SIZE + 2)
        throw Error(ErrorCode::InvalidData, "The selected BGCode file is truncated: " + path);

    const unsigned char major = static_cast<unsigned char>(content[BGCODE_MAJOR_VERSION_OFFSET]);
    const unsigned char minor = static_cast<unsigned char>(content[BGCODE_MINOR_VERSION_OFFSET]);
    if (major != SUPPORTED_MAJOR_VERSION) {
        throw Error(
            ErrorCode::UnsupportedVersion,
            "Unsupported BGCode version " + std::to_string(major) + "." + std::to_string(minor) + " in file: " + path);
    }

    const std::string payload = content.substr(BGCODE_MAGIC_SIZE + 2);
    if (payload.empty())
        throw Error(ErrorCode::InvalidData, "The selected BGCode file has no G-code payload: " + path);
    if (contains_nul(payload)) {
        throw Error(
            ErrorCode::UnsupportedEncoding,
            "The selected BGCode file uses an unsupported payload encoding: " + path);
    }

    const boost::filesystem::path tmp_path =
        boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("orcaslicer-bgcode-%%%%%%%%.gcode");
    write_all(tmp_path.string(), payload);
    return { path, tmp_path.string(), true };
}

void copy_text_to_bgcode_file(const std::string& text_gcode_path, const std::string& bgcode_path)
{
    if (!boost::filesystem::exists(text_gcode_path))
        throw Error(ErrorCode::IoError, "Source G-code file does not exist: " + text_gcode_path);

    const std::string text_payload = read_all(text_gcode_path);
    std::string       encoded;
    encoded.reserve(BGCODE_MAGIC_SIZE + 2 + text_payload.size());
    encoded.append(BGCODE_MAGIC, BGCODE_MAGIC_SIZE);
    encoded.push_back(static_cast<char>(SUPPORTED_MAJOR_VERSION));
    encoded.push_back(static_cast<char>(SUPPORTED_MINOR_VERSION));
    encoded.append(text_payload);
    write_all(bgcode_path, encoded);
}

} // namespace Slic3r::BinaryGCode
