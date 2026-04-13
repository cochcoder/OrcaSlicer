#ifndef slic3r_BinaryGCode_hpp_
#define slic3r_BinaryGCode_hpp_

#include <stdexcept>
#include <string>

namespace Slic3r::BinaryGCode {

enum class ErrorCode : unsigned char
{
    IoError,
    UnsupportedVersion,
    UnsupportedEncoding,
    InvalidData
};

class Error : public std::runtime_error
{
public:
    Error(ErrorCode code, const std::string& message);
    ErrorCode code() const { return m_code; }

private:
    ErrorCode m_code;
};

struct DetectionResult
{
    bool is_binary_gcode { false };
    int  major_version { -1 };
    int  minor_version { -1 };
    bool looks_binary_payload { false };
};

struct NormalizedInput
{
    std::string source_path;
    std::string parser_path;
    bool        uses_temporary_path { false };
};

bool is_binary_gcode_path(const std::string& path);
DetectionResult detect_file(const std::string& path);
NormalizedInput normalize_for_gcode_parser(const std::string& path);
void copy_text_to_bgcode_file(const std::string& text_gcode_path, const std::string& bgcode_path);

} // namespace Slic3r::BinaryGCode

#endif // slic3r_BinaryGCode_hpp_
