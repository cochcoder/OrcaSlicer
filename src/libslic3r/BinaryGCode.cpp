#include "BinaryGCode.hpp"

#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <boost/nowide/fstream.hpp>
#include <algorithm>

namespace Slic3r::BinaryGCode {

namespace {

constexpr size_t PROBE_SIZE = 4096;
// Header probe currently expects: "BGCODE" + 2 bytes (major/minor version).
constexpr char BGCODE_MAGIC[] = "BGCODE";
constexpr size_t BGCODE_MAGIC_SIZE = sizeof(BGCODE_MAGIC) - 1;

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
        ret.major_version = static_cast<unsigned char>(probe[BGCODE_MAGIC_SIZE + 0]);
        ret.minor_version = static_cast<unsigned char>(probe[BGCODE_MAGIC_SIZE + 1]);
    }

    return ret;
}

NormalizedInput normalize_for_gcode_parser(const std::string& path)
{
    const DetectionResult detection = detect_file(path);
    if (!detection.is_binary_gcode)
        return { path, path, false };

    if (detection.looks_binary_payload) {
        throw Error(
            ErrorCode::UnsupportedEncoding,
            "The selected .bgcode file contains binary payload that is not yet supported by this build.");
    }

    return { path, path, false };
}

void copy_text_to_bgcode_file(const std::string& text_gcode_path, const std::string& bgcode_path)
{
    if (!boost::filesystem::exists(text_gcode_path))
        throw Error(ErrorCode::IoError, "Source G-code file does not exist: " + text_gcode_path);

    boost::filesystem::copy_file(
        boost::filesystem::path(text_gcode_path),
        boost::filesystem::path(bgcode_path),
        boost::filesystem::copy_option::overwrite_if_exists);
}

} // namespace Slic3r::BinaryGCode
