#include <sapphire/ring_buffer.hpp>

#include <cstdlib>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

namespace {

[[noreturn]] void fail(const std::string& message) {
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

void expect(bool condition, const std::string& message) {
    if (!condition) {
        fail(message);
    }
}

void testLinearAccess() {
    sapphire::RingBuffer<int, 4> buffer;
    expect(buffer.empty(), "new buffer must be empty");

    buffer.push_back(1);
    buffer.push_back(2);
    buffer.push_back(3);

    expect(buffer.size() == 3, "linear size");
    expect(buffer.front() == 1, "linear front");
    expect(buffer.back() == 3, "linear back");
    expect(buffer[1] == 2, "linear random access");
}

void testWrapOverwritesOldest() {
    sapphire::RingBuffer<int, 4> buffer;
    for (int value = 1; value <= 6; ++value) {
        buffer.push_back(value);
    }

    const std::vector<int> expected{3, 4, 5, 6};
    expect(buffer.full(), "wrapped buffer must remain full");
    for (std::size_t i = 0; i < expected.size(); ++i) {
        expect(buffer[i] == expected[i], "wrapped logical order");
    }
}

void testClearResetsLogicalState() {
    sapphire::RingBuffer<int, 3> buffer;
    buffer.push_back(1);
    buffer.push_back(2);
    buffer.push_back(3);
    buffer.push_back(4);
    buffer.clear();
    buffer.push_back(9);

    expect(buffer.size() == 1, "clear must reset size");
    expect(buffer.front() == 9 && buffer.back() == 9,
           "push after clear must establish a new sequence");
}

void testConstAndReverseIteration() {
    sapphire::RingBuffer<int, 3> buffer;
    buffer.push_back(1);
    buffer.push_back(2);
    buffer.push_back(3);
    buffer.push_back(4);
    const auto& const_buffer = buffer;

    const std::vector<int> forward(
        const_buffer.begin(), const_buffer.end());
    const std::vector<int> reverse(
        const_buffer.rbegin(), const_buffer.rend());

    expect(forward == std::vector<int>({2, 3, 4}),
           "const iteration order");
    expect(reverse == std::vector<int>({4, 3, 2}),
           "const reverse iteration order");
    expect(std::distance(const_buffer.end(), const_buffer.begin()) == -3,
           "iterator difference must preserve sign");
}

void testCopyIsIndependentSnapshot() {
    sapphire::RingBuffer<int, 3> original;
    original.push_back(1);
    original.push_back(2);
    auto snapshot = original;
    original.push_back(3);

    expect(snapshot.size() == 2, "snapshot size must be independent");
    expect(snapshot.back() == 2, "snapshot contents must be independent");
}

}  // namespace

int main() {
    testLinearAccess();
    testWrapOverwritesOldest();
    testClearResetsLogicalState();
    testConstAndReverseIteration();
    testCopyIsIndependentSnapshot();
    std::cout << "All ring buffer tests passed\n";
    return EXIT_SUCCESS;
}
