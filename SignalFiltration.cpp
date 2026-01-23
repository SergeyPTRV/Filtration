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

    cout << "--- Starting Data Stream (Pull Model) ---\n";

    // We pass the function POINTERS: getNextVoltage and checkVoltageThreshold

    processSensorStream(smaFilter, getNextVoltage, checkVoltageThreshold); // Process with SMA
    
    resetSensorStream(); // Reset for next filter
    
    processSensorStream(emaFilter, getNextVoltage, checkVoltageThreshold); // Process with EMA

    return 0;
}
