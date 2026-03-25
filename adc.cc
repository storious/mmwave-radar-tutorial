module;

#include "config.hpp"

export module mmwave.radar.frame_processor;

import std;

export namespace mmwave::radar {

// ======================================================================================
// Strongly typed radar configuration (compile-time fixed)
// ======================================================================================
struct RadarConfig {
    static constexpr size_t samples_per_chip  = NumADCSamples;
    static constexpr size_t rx_channels       = NumRx;
    static constexpr size_t chirps_per_frame  = ChirpsPerFrame;
    static constexpr size_t total_samples     = chirps_per_frame * rx_channels * samples_per_chip;
    static constexpr size_t frame_bytes       = total_samples * sizeof(std::complex<int16_t>);
};

// ======================================================================================
// Strong type aliases
// ======================================================================================
using RawBytes      = std::vector<std::byte>;
using RawInt16      = std::vector<int16_t>;
using ComplexI16    = std::complex<int16_t>;
using ADCFrame      = std::vector<ComplexI16>;

using FrameView     = std::mdspan<ComplexI16, std::dextents<size_t, 3>, std::layout_right>;
using ConstFrameView= std::mdspan<const ComplexI16, std::dextents<size_t, 3>, std::layout_right>;

// ======================================================================================
// Error code enumeration (no exceptions)
// ======================================================================================
enum class FrameError {
    NoFilesAdded,
    FilesNotOpen,
    FileOpenFailed,
    InvalidFrameSize,
    ReadIncomplete,
    InvalidLVDSData,
    DataMismatch,
    EndOfFile
};

// ======================================================================================
// std::expected aliases for clean API
// ======================================================================================
template <typename T>
using Result = std::expected<T, FrameError>;

using VoidResult = Result<std::monostate>;

// ======================================================================================
// TI LVDS Format Converter (pluggable strategy)
// ======================================================================================
struct TI_LVDS_Converter {
    static constexpr size_t stride = 4;

    [[nodiscard]] static Result<ADCFrame> convert(const RawInt16& raw) noexcept {
        if (raw.size() % stride != 0) {
            return std::unexpected(FrameError::InvalidLVDSData);
        }

        ADCFrame frame(raw.size() / 2);
        size_t idx = 0;

        for (size_t i = 0; i + 3 < raw.size(); i += stride) {
            frame[idx++] = ComplexI16{raw[i],     raw[i+2]};
            frame[idx++] = ComplexI16{raw[i+1],   raw[i+3]};
        }

        return frame;
    }
};

using DefaultConverter = TI_LVDS_Converter;

// ======================================================================================
// RAII Frame Reader (no exceptions, std::expected only)
// ======================================================================================
export class FrameReader {
public:
    FrameReader() = default;
    ~FrameReader() { close(); }

    FrameReader(const FrameReader&) = delete;
    FrameReader& operator=(const FrameReader&) = delete;
    FrameReader(FrameReader&&) = default;
    FrameReader& operator=(FrameReader&&) = default;

    void add_file(std::string path) noexcept {
        _paths.push_back(std::move(path));
        _opened = false;
    }

    void add_files(std::vector<std::string> paths) noexcept {
        std::move(paths.begin(), paths.end(), std::back_inserter(_paths));
        _opened = false;
    }

    [[nodiscard]] VoidResult open() noexcept {
        if (_paths.empty()) {
            return std::unexpected(FrameError::NoFilesAdded);
        }

        _streams.clear();
        for (const auto& p : _paths) {
            std::ifstream fs(p, std::ios::binary);
            if (!fs) {
                return std::unexpected(FrameError::FileOpenFailed);
            }
            _streams.push_back(std::move(fs));
        }

        _opened = true;
        return {};
    }

    void close() noexcept {
        for (auto& s : _streams) {
            if (s.is_open()) s.close();
        }
        _opened = false;
    }

    [[nodiscard]] Result<RawBytes> read_raw() noexcept {
        if (!_opened) {
            return std::unexpected(FrameError::FilesNotOpen);
        }

        RawBytes buf(RadarConfig::frame_bytes);
        auto n = read_bytes(buf.data(), buf.size());

        if (n != buf.size()) {
            return std::unexpected(n == 0 ? FrameError::EndOfFile : FrameError::ReadIncomplete);
        }

        return buf;
    }

    [[nodiscard]] Result<ADCFrame> read_frame() noexcept {
        auto raw = read_raw();
        if (!raw) return std::unexpected(raw.error());

        auto i16 = bytes_to_i16(*raw);
        return DefaultConverter::convert(i16);
    }

    [[nodiscard]] bool is_open() const noexcept { return _opened; }
    [[nodiscard]] size_t file_count() const noexcept { return _paths.size(); }

private:
    [[nodiscard]] RawInt16 bytes_to_i16(const RawBytes& b) const noexcept {
        RawInt16 res(b.size() / sizeof(int16_t));
        std::memcpy(res.data(), b.data(), b.size());
        return res;
    }

    [[nodiscard]] size_t read_bytes(std::byte* dst, size_t n) noexcept {
        size_t done = 0;
        for (auto& s : _streams) {
            size_t rem = n - done;
            if (rem == 0) break;
            s.read(reinterpret_cast<char*>(dst + done), rem);
            done += s.gcount();
        }
        return done;
    }

private:
    std::vector<std::string> _paths;
    std::vector<std::ifstream> _streams;
    bool _opened = false;
};

// ======================================================================================
// Zero-Copy Frame Processor (mdspan view)
// ======================================================================================
export class FrameProcessor {
public:
    [[nodiscard]] static Result<FrameView> make_view(ADCFrame& frame) noexcept {
        if (frame.size() != RadarConfig::total_samples) {
            return std::unexpected(FrameError::DataMismatch);
        }
        return FrameView{
            frame.data(),
            RadarConfig::chirps_per_frame,
            RadarConfig::rx_channels,
            RadarConfig::samples_per_chip
        };
    }

    [[nodiscard]] static Result<ConstFrameView> make_const_view(const ADCFrame& frame) noexcept {
        if (frame.size() != RadarConfig::total_samples) {
            return std::unexpected(FrameError::DataMismatch);
        }
        return ConstFrameView{
            frame.data(),
            RadarConfig::chirps_per_frame,
            RadarConfig::rx_channels,
            RadarConfig::samples_per_chip
        };
    }

    static void print_info(ConstFrameView v) noexcept {
        std::println("Radar Frame: {} chirps | {} rx | {} samples | total: {}",
            v.extent(0), v.extent(1), v.extent(2), v.size());
    }
};

} // namespace mmwave::radar
