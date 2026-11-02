#include <H5Cpp.h>
#include <H5Zpublic.h>
#include <opencv2/core.hpp>

#include <charconv>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "flow.h"

namespace {

struct EventData {
    std::vector<std::uint16_t> x;
    std::vector<std::uint16_t> y;
    std::vector<std::uint32_t> t;
    std::vector<std::uint8_t> p;
    std::vector<std::uint64_t> ms_to_idx;
    std::int64_t t_offset{};
};

struct SequenceInterval {
    std::uint64_t from_timestamp_us;
    std::uint64_t to_timestamp_us;
    std::uint64_t file_index;
};

struct DsecData {
    EventData events;
    cv::Mat rectify_map;
    std::vector<SequenceInterval> intervals;
};

void checkAvailableFilters(const H5::DataSet& dataset, const std::string& dataset_name)
{
    const H5::DSetCreatPropList properties = dataset.getCreatePlist();
    for (int i = 0; i < properties.getNfilters(); ++i) {
        unsigned int flags{};
        unsigned int values[32]{};
        unsigned int filter_config{};
        char filter_name[128]{};
        size_t value_count = 32;
        const H5Z_filter_t filter = properties.getFilter(
            i, flags, value_count, values, sizeof(filter_name), filter_name, filter_config);
        if (H5Zfilter_avail(filter) <= 0) {
            throw std::runtime_error(
                dataset_name + " requires unavailable HDF5 filter " +
                (filter_name[0] == '\0' ? std::to_string(filter) : filter_name) +
                " (ID " + std::to_string(filter) + "). Install a compatible HDF5 "
                "filter plugin and set HDF5_PLUGIN_PATH.");
        }
    }
}

template <typename T>
std::vector<T> readVector(
    const H5::H5File& file,
    const std::string& dataset_name,
    const H5::PredType& native_type)
{
    const H5::DataSet dataset = file.openDataSet(dataset_name);
    checkAvailableFilters(dataset, dataset_name);
    const H5::DataSpace space = dataset.getSpace();
    if (space.getSimpleExtentNdims() != 1) {
        throw std::runtime_error(dataset_name + " must be a one-dimensional dataset");
    }

    hsize_t dimensions[1]{};
    space.getSimpleExtentDims(dimensions);
    if (dimensions[0] > std::numeric_limits<std::size_t>::max()) {
        throw std::runtime_error(dataset_name + " is too large for this platform");
    }

    std::vector<T> values(static_cast<std::size_t>(dimensions[0]));
    if (!values.empty()) {
        dataset.read(values.data(), native_type);
    }
    return values;
}

EventData loadEvents(const std::string& path)
{
    const H5::H5File file(path, H5F_ACC_RDONLY);
    EventData events;
    events.x = readVector<std::uint16_t>(file, "/events/x", H5::PredType::NATIVE_UINT16);
    events.y = readVector<std::uint16_t>(file, "/events/y", H5::PredType::NATIVE_UINT16);
    events.t = readVector<std::uint32_t>(file, "/events/t", H5::PredType::NATIVE_UINT32);
    events.p = readVector<std::uint8_t>(file, "/events/p", H5::PredType::NATIVE_UINT8);
    events.ms_to_idx =
        readVector<std::uint64_t>(file, "/ms_to_idx", H5::PredType::NATIVE_UINT64);

    const H5::DataSet offset_dataset = file.openDataSet("/t_offset");
    checkAvailableFilters(offset_dataset, "/t_offset");
    if (offset_dataset.getSpace().getSimpleExtentNdims() != 0) {
        throw std::runtime_error("/t_offset must be a scalar dataset");
    }
    offset_dataset.read(&events.t_offset, H5::PredType::NATIVE_INT64);

    if (events.x.size() != events.y.size() ||
        events.x.size() != events.t.size() ||
        events.x.size() != events.p.size()) {
        throw std::runtime_error("event datasets do not contain the same number of events");
    }
    return events;
}

cv::Mat loadRectifyMap(const std::string& path)
{
    const H5::H5File file(path, H5F_ACC_RDONLY);
    const H5::DataSet dataset = file.openDataSet("/rectify_map");
    checkAvailableFilters(dataset, "/rectify_map");
    const H5::DataSpace space = dataset.getSpace();
    if (space.getSimpleExtentNdims() != 3) {
        throw std::runtime_error("/rectify_map must have shape [height, width, 2]");
    }

    hsize_t dimensions[3]{};
    space.getSimpleExtentDims(dimensions);
    if (dimensions[2] != 2 ||
        dimensions[0] == 0 || dimensions[1] == 0 ||
        dimensions[0] > static_cast<hsize_t>(std::numeric_limits<int>::max()) ||
        dimensions[1] > static_cast<hsize_t>(std::numeric_limits<int>::max())) {
        throw std::runtime_error("/rectify_map must have a valid [height, width, 2] shape");
    }

    cv::Mat map(
        static_cast<int>(dimensions[0]),
        static_cast<int>(dimensions[1]),
        CV_32FC2);
    dataset.read(map.ptr<float>(), H5::PredType::NATIVE_FLOAT);
    return map;
}

std::string trim(const std::string& value)
{
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::uint64_t parseUnsigned(const std::string& value, std::size_t line_number)
{
    const std::string field = trim(value);
    std::uint64_t parsed{};
    const auto result = std::from_chars(field.data(), field.data() + field.size(), parsed);
    if (field.empty() || result.ec != std::errc{} || result.ptr != field.data() + field.size()) {
        throw std::runtime_error("invalid integer in CSV line " + std::to_string(line_number));
    }
    return parsed;
}

std::vector<SequenceInterval> loadIntervals(const std::string& path)
{
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("could not open CSV file: " + path);
    }

    std::string line;
    if (!std::getline(input, line)) {
        throw std::runtime_error("CSV file is empty: " + path);
    }

    std::vector<SequenceInterval> intervals;
    std::size_t line_number = 1;
    while (std::getline(input, line)) {
        ++line_number;
        if (trim(line).empty()) {
            continue;
        }

        std::istringstream row(line);
        std::string from;
        std::string to;
        std::string index;
        std::string extra;
        if (!std::getline(row, from, ',') ||
            !std::getline(row, to, ',') ||
            !std::getline(row, index, ',') ||
            std::getline(row, extra, ',')) {
            throw std::runtime_error("expected three CSV columns on line " +
                                     std::to_string(line_number));
        }

        SequenceInterval interval{
            parseUnsigned(from, line_number),
            parseUnsigned(to, line_number),
            parseUnsigned(index, line_number)};
        if (interval.to_timestamp_us < interval.from_timestamp_us) {
            throw std::runtime_error("CSV interval ends before it starts on line " +
                                     std::to_string(line_number));
        }
        intervals.push_back(interval);
    }
    if (!input.eof()) {
        throw std::runtime_error("error while reading CSV file: " + path);
    }
    return intervals;
}

DsecData loadData(const std::string& events_path,
                  const std::string& rectify_map_path,
                  const std::string& intervals_path)
{
    DsecData data;
    data.events = loadEvents(events_path);
    data.rectify_map = loadRectifyMap(rectify_map_path);
    data.intervals = loadIntervals(intervals_path);
    return data;
}

} // namespace

int main(int argc, char* argv[])
{
    if (argc > 4) {
        std::cerr << "Usage: " << argv[0]
                  << " [events_first10pct.h5] [rectify_map.h5] "
                     "[interlaken_00_b.csv]\n";
        return 2;
    }

    const std::string events_path =
        argc > 1 ? argv[1] : "../sampledata/events_first10pct.h5";
    const std::string rectify_map_path =
        argc > 2 ? argv[2] : "../sampledata/rectify_map.h5";
    const std::string intervals_path =
        argc > 3 ? argv[3] : "../sampledata/interlaken_00_b.csv";

    cv::namedWindow("Flow", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    std::cout << "Loading events from " << events_path << ", rectify map from "
              << rectify_map_path << ", and intervals from " << intervals_path
              << "...\n";
    DsecData data;
    try {
        data = loadData(events_path, rectify_map_path, intervals_path);
        std::cout << "Loaded " << data.events.x.size() << " events, "
                  << data.events.ms_to_idx.size() << " millisecond indices, "
                  << "rectification map " << data.rectify_map.cols << 'x'
                  << data.rectify_map.rows << ", and " << data.intervals.size()
                  << " sequence intervals.\n";
    } catch (const H5::Exception& error) {
        std::cerr << "HDF5 load failed: " << error.getDetailMsg() << '\n';
        return 1;
    } catch (const std::exception& error) {
        std::cerr << "Data load failed: " << error.what() << '\n';
        return 1;
    }

    cv::Mat blank_img(data.rectify_map.rows, data.rectify_map.cols, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Mat events_img(data.rectify_map.rows, data.rectify_map.cols, CV_8UC3, cv::Scalar(255, 255, 255));
    zrtFlow flow;
    flow.initialise(data.rectify_map.size(), 20, 200, 2, 20, 0.125, 3);
    for (std::size_t i = 0; i < data.events.x.size(); ++i) {
        const std::uint16_t x = data.events.x[i];
        const std::uint16_t y = data.events.y[i];
        const double t = data.events.t[i]*1e-6;
        
        // Process this event using x, y, and t.
        flow.add(x, y, t);
        events_img.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0); // Mark the event on the image
        if (i % 100000 == 0) {
            flow.update();
            cv::Mat flow_image = blank_img - flow.makebgr();
            cv::imshow("Flow", flow_image);
            cv::imshow("Events", events_img);
            if (cv::waitKey(3) == 27) { // ESC key
                break;  
            }
            events_img.setTo(cv::Scalar(255, 255, 255)); // Reset the events image
        }
    }

    return 0;
}