#include <iostream>
#include <iomanip> // For pretty printing
using namespace std;

// --- Existing Interface and Filter (AccurateSMA) ---
template <typename T>
class IFilter {
public:
    virtual ~IFilter() {}
    virtual T filter(T input) = 0;
};

template <typename T, size_t N>
class AccurateSMA : public IFilter<T> {
public:
    T filter(T input) override {
        // Subtract the oldest value from the running sum
        _sum -= _buffer[_index];
        // Overwrite the oldest value with the new input
        _buffer[_index] = input;
        // Add the new input to the running sum
        _sum += input;
        // Advance the index (circularly)
        _index = (_index + 1) % N;
        // "Growing Window" logic: count samples until the buffer is full
        if (_count < N) _count++;
        // Return the average
        return _sum / static_cast<T>(_count);
    }
private:
    T _buffer[N] = {0}; 
    T _sum = 0; 
    size_t _index = 0; 
    size_t _count = 0;
};


// --- Exponential Moving Average (EMA) ---
template <typename T, size_t N>
class FilterEMA : public IFilter<T> {
public:
    FilterEMA() : _filteredValue(0), _initialized(false) {
        // Calculate _alpha based on the window size N
        // Standard formula: _alpha = 2 / (N + 1)
        if (N > 0) {
            _alpha = static_cast<T>(2.0) / static_cast<T>(N + 1);
        } else {
            _alpha = static_cast<T>(1.0); // No filtering if N=0
        }
    }

    T filter(T input) override {
        if (!_initialized) {
            _filteredValue = input;
            _initialized = true;
        } else {
            // EMA Difference Equation: y[n] = _alpha * x[n] + (1 - _alpha) * y[n-1]
            _filteredValue = (_alpha * input) + (static_cast<T>(1.0) - _alpha) * _filteredValue;
        }
        return _filteredValue;
    }

    // Helper to reset the filter if the sensor is power-cycled
    void reset() {
        _initialized = false;
    }

private:
    T _filteredValue;
    T _alpha;
    bool _initialized;

};

#if __STDC_HOSTED__ == 1 && __cplusplus >= 201402L
#include <array>
#include <algorithm>

// --- Median Filter Implementation for full OS and Standard Library (Windows/Linux/macOS) ---
template <typename T, size_t N>
class FilterMedian : public IFilter<T> {
    static_assert(N > 0, "Filter size must be greater than 0");

public:
    FilterMedian() : _head(0), _is_full(false) {
        _buffer.fill(0);
    }

    T filter(T input) override {
        // 1. Update circular buffer
        _buffer[_head] = input;
        _head = (_head + 1) % N;
        if (_head == 0) _is_full = true;

        // 2. Copy to local array for sorting (to keep original order in _buffer)
        std::array<T, N> sort_buffer = _buffer;
        
        // 3. Find the median
        // If the buffer isn't full yet, sort the available samples only 
        size_t current_size = _is_full ? N : _head;
        auto begin = sort_buffer.begin();
        auto end = begin + current_size;
        auto median_it = begin + (current_size / 2);

        // std::nth_element is faster than std::sort for finding medians
        std::nth_element(begin, median_it, end);

        return *median_it;
    }

    void reset() {
        _head = 0;
        _is_full = false;
        _buffer.fill(0);
    }

private:
    std::array<T, N> _buffer;
    size_t _head;
    bool _is_full;
};

#else

// --- Median Filter Implementation like bare-metal(No std library) ---
template <typename T, size_t N>
class FilterMedian : public IFilter<T> {
public:
    FilterMedian() : _head(0), _is_full(false) {
        for (size_t i = 0; i < N; ++i) {
            _buffer[i] = 0;
        }
    }

    T filter(T input) override {
        // 1. Update circular buffer
        _buffer[_head] = input;
        _head = (_head + 1) % N;
        if (_head == 0) _is_full = true;

        // 2. Determine current valid size
        size_t current_size = _is_full ? N : _head;

        // 3. Copy to local array for sorting (to keep original order in _buffer)
        T sort_buffer[N];
        for (size_t i = 0; i < current_size; ++i) {
            sort_buffer[i] = _buffer[i];
        }

        // 4. Manual Insertion Sort
        insertion_sort(sort_buffer, current_size);

        // 5. Return the middle element
        return sort_buffer[current_size / 2];
    }

    void reset() {
        _head = 0;
        _is_full = false;
        for (size_t i = 0; i < N; ++i) {
            _buffer[i] = 0;
        }
    }

private:
    T _buffer[N];
    size_t _head;
    bool _is_full;

    // Self-implemented stable insertion sort
    void insertion_sort(T* arr, size_t n) {
        for (size_t i = 1; i < n; ++i) {
            T key = arr[i];
            int j = (int)i - 1;
            while (j >= 0 && arr[j] > key) {
                arr[j + 1] = arr[j];
                j = j - 1;
            }
            arr[j + 1] = key;
        }
    }
};

#endif


// --- New Types ---
typedef float (*DataSource)();         // Function pointer for Input
typedef void (*ProcessingCallback)(float); // Function pointer for Output

// --- Data Provider Logic ---
// We use a static array and index to simulate a hardware buffer
static float rawSensors[] = {12.0, 12.2, 11.9, 14.5, 12.1, 12.0, 11.8, 15.2, 12.3, 12.1};
static size_t sensorIndex = 0;

float getNextVoltage() {
    if (sensorIndex < 10) {
        return rawSensors[sensorIndex++];
    }
    return -1.0f; // Signal "End of Data"
}

void resetSensorStream() {
    cout << "\n--- Resetting Sensor Stream ---\n";
    sensorIndex = 0;
}

// --- The Core Processor ---
// Now it "Pulls" the data from the provider function
void processSensorStream(IFilter<float>& filter, DataSource source, ProcessingCallback outputAction) {
    if (!source || !outputAction) return;

    while (true) {
        float rawVal = source(); // Pulling data
        if (rawVal < 0) break;   // Exit condition

        float cleanVal = filter.filter(rawVal);
        outputAction(cleanVal);  // Executing action
    }
}

// --- Output Action ---
void checkVoltageThreshold(float value) {
    cout << "Filtered Output: " << value << (value < 11.5f ? " [LOW]" : "") << "\n";
}

// --- Main Function ---
// Demonstrates usage of both filters
int main() 
{

    AccurateSMA<float, 5> smaFilter; // 5-sample SMA filter
    FilterEMA<float, 5> emaFilter; // Filters noise using a 5-sample window
    FilterMedian<float, 5> medianFilter; // 5-sample Median filter

    cout << "--- Starting Data Stream (Pull Model) ---\n";

    // We pass the function POINTERS: getNextVoltage and checkVoltageThreshold

    processSensorStream(smaFilter, getNextVoltage, checkVoltageThreshold); // Process with SMA
    
    resetSensorStream(); // Reset for next filter
    
    processSensorStream(emaFilter, getNextVoltage, checkVoltageThreshold); // Process with EMA

    resetSensorStream(); // Reset for next filter
    
    processSensorStream(medianFilter, getNextVoltage, checkVoltageThreshold); // Process with Median

    return 0;
}
