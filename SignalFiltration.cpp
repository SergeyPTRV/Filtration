#include <iostream>
#include <iomanip> // For pretty printing
using namespace std;

#pragma region Filter Interface
// --- Existing Interface and Filter (AccurateSMA) ---
template <typename T>
class IFilter {
public:
    virtual ~IFilter() {}
    virtual T filter(T input) = 0;
};
#pragma endregion   
#pragma region SMA Filter
// --- Simple Moving Average (SMA) with "Growing Window" logic ---
template <typename T, size_t N>
class FilterSMA : public IFilter<T> {
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
#pragma endregion
#pragma region EMA Filter
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
#pragma endregion
#pragma region Median Filter
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
#pragma endregion
#pragma region Kalman Filter
#define Kalman_One_Direction
#ifdef Kalman_One_Direction
// One-dimensional Kalman Filter Implementation
template <typename T, size_t N>
class FilterKalman : public IFilter<T> {
    // Ensure this implementation is only used for the 1D case
    static_assert(N == 1, "This implementation is specialized for N=1 (scalar filter)");

private:
    T x; // State estimate
    T p; // Estimation error covariance
    T q; // Process noise covariance
    T r; // Measurement noise covariance
    T k; // Kalman Gain

public:
    /**
     * Constructor
     * @param process_noise (Q): Model trust (lower values result in smoother output)
     * @param sensor_noise (R): Sensor trust (higher values result in stronger filtering)
     * @param initial_value: The starting estimate for the signal
     */
    FilterKalman(T process_noise, T sensor_noise, T initial_value) 
        : q(process_noise), r(sensor_noise), x(initial_value), p(static_cast<T>(1.0)) 
    {}

    // Implementation of the primary filtering method
    T filter(T input) override {
        // --- Step 1: Predict ---
        // In 1D, we assume the state remains constant, but uncertainty (p) grows by q
        p = p + q;

        // --- Step 2: Update (Correct) ---
        // Calculate the Kalman Gain
        k = p / (p + r);

        // Update the state estimate with the new measurement
        x = x + k * (input - x);

        // Update the error covariance (uncertainty decreases after measurement)
        p = (static_cast<T>(1.0) - k) * p;

        return x;
    }

    // Helper method to reset the filter state
    void reset(T initial_value) {
        x = initial_value;
        p = static_cast<T>(1.0);
    }
};

#else
/**
 * Multivariate Kalman Filter
 * T: The data type (float/double)
 * N: The number of states (e.g., 2 for Position & Velocity)
 */
template <typename T, size_t N>
class FilterKalman : public IFilter<T> {
public:
    FilterKalman(T process_noise, T sensor_noise, T initial_value)
        : _process_noise(process_noise), _sensor_noise(sensor_noise), _initial_value(initial_value) {
        _R = _sensor_noise;
        for (size_t i = 0; i < N; i++) {
            _x[i] = (i == 0) ? _initial_value : 0;
            _H[i] = (i == 0) ? 1.0 : 0.0; // We observe the first state
            for (size_t j = 0; j < N; j++) {
                _F[i][j] = (i == j) ? 1.0 : 0.0;
                _Q[i][j] = (i == j) ? _process_noise : 0.0;
                _P[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
    }

    T filter(T input) override {
        // 1. Predict: x = F * x
        // (Simplified 1D-transition logic for example)
        T x_pred[N];
        for (size_t i = 0; i < N; i++) {
            x_pred[i] = 0;
            for (size_t j = 0; j < N; j++) {
                x_pred[i] += _F[i][j] * _x[j];
            }
        }

        // 2. Predict Covariance: P = F * P * F' + Q
        // (Matrix math omitted for brevity, but crucial for N > 1)

        // 3. Calculate Kalman Gain: K = P * H' / (H * P * H' + R)
        T innovation_covariance = 0;
        for (size_t i = 0; i < N; i++) {
            innovation_covariance += _H[i] * _P[i][i] * _H[i];
        }
        innovation_covariance += _R;

        for (size_t i = 0; i < N; i++) {
            _K[i] = (_P[i][i] * _H[i]) / innovation_covariance;
        }

        // 4. Update State: x = x_pred + K * (input - H * x_pred)
        T residual = input - x_pred[0]; // Assuming measurement corresponds to state 0
        for (size_t i = 0; i < N; i++) {
            _x[i] = x_pred[i] + _K[i] * residual;
        }

        // 5. Update Covariance: P = (I - K * H) * P
        for (size_t i = 0; i < N; i++) {
            _P[i][i] = (1.0 - _K[i] * _H[i]) * _P[i][i];
        }

        return _x[0]; // Returning the primary state estimate
    }

private:
    // State Matrices
    T _x[N];       // State estimate vector
    T _P[N][N];    // Estimate covariance matrix (Uncertainty)
    T _Q[N][N];    // Process noise covariance
    T _R;          // Measurement noise (Scalar for a single sensor input)
    T _K[N];       // Kalman Gain vector
    T _H[N];       // Observation model (Maps state to measurement)
    T _F[N][N];    // State transition model (Physics model)

    T _process_noise;   //(Q): Model trust (lower values result in smoother output)
    T _sensor_noise;    //(R): Sensor trust (higher values result in stronger filtering)
    T _initial_value;   //The starting estimate for the signal
};
#endif
#pragma endregion

#pragma region Core Processor
// --- New Types ---
typedef float (*DataSource)();         // Function pointer for Input
typedef void (*ProcessingCallback)(float); // Function pointer for Output

// "Pulls" the data from the provider function
void processSensorStream(IFilter<float>& filter, DataSource source, ProcessingCallback outputAction) {
    if (!source || !outputAction) return;

    while (true) {
        float rawVal = source(); // Pulling data
        if (rawVal < 0) break;   // Exit condition

        float cleanVal = filter.filter(rawVal);
        outputAction(cleanVal);  // Executing action
    }
}
#pragma endregion

#pragma region Auxiliary functions
// --- Data Provider Logic ---
// We use a static array and index to simulate a hardware buffer
static float rawSensors[] = {12.0, 12.2, 11.9, 14.5, 15.1, 
                            15.0, 15.8, 15.2, 15.3, 15.1, 
                            15.0, 15.2, 15.9, 15.5, 15.1, 
                            15.0, 15.8, 15.2, 15.3, 15.1};
static size_t sensorIndex = 0;

#define RAW_DATA_SIZE (sizeof(rawSensors) / sizeof(rawSensors[0]))   

float getNextVoltage() {
    if (sensorIndex < RAW_DATA_SIZE) {
        return rawSensors[sensorIndex++];
    }
    return -1.0f; // Signal "End of Data"
}

// --- Output Action ---
void checkVoltageThreshold(float value) {
    cout << "Filtered Output: " << value << (value < 11.5f ? " [LOW]" : "") << "\n";
}

void resetSensorStream() {
    cout << "\n--- Resetting Sensor Stream ---\n";
    sensorIndex = 0;
}
#pragma endregion

// --- Main Function ---
// Demonstrates usage of the filters
int main() 
{
    // 1. Initialize specific filter instances on the stack
    FilterSMA<float, 5> smaFilter; // 5-sample SMA filter
    FilterEMA<float, 5> emaFilter; // 5-sample window EMA filter
    FilterMedian<float, 5> medianFilter; // 5-sample Median filter
 
    const float Q = 0.1f;
    const float R = 0.1f;
    const float initial_guess = 12.0f;
    FilterKalman<float, 1> kalmanFilter(Q, R, initial_guess);

    // 2. Organize filters into an array of interface pointers
    // This works because all classes inherit from IFilter<float>
    IFilter<float>* filters[] = {
        &smaFilter,
        &emaFilter,
        &medianFilter,
        &kalmanFilter
    };

    // Parallel array for descriptive logging
    const std::string filterNames[] = {
        "SMA Filter",
        "EMA Filter",
        "Median Filter",
        "Kalman Filter (1D)"
    };
    
    cout << "--- Starting Data Stream (Automated Pull Model) ---\n";

    // 3. Iterate through the array to process each filter
    const size_t numFilters = sizeof(filters) / sizeof(filters[0]);

    for (size_t i = 0; i < numFilters; ++i) {
        cout << "\n>>> Testing Strategy: " << filterNames[i] << "\n";
        
        // Dereference the pointer to pass the object by reference
        // We pass the function POINTERS: getNextVoltage and checkVoltageThreshold
        processSensorStream(*filters[i], getNextVoltage, checkVoltageThreshold);
        
        // Prepare the generator for the next filter iteration
        resetSensorStream(); 
    }

    cout << "\n--- All filtering strategies completed ---\n";

    return 0;
}
