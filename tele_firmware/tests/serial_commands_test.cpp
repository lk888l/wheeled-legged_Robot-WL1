#include "SerialCommandQueue.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <string>

using Queue = SerialCommandQueue;

static Queue::Command take(Queue& queue, Queue::Kind kind)
{
    Queue::Command command;
    assert(queue.pop(command));
    assert(command.kind == kind);
    return command;
}

static std::string text(const Queue::Command& command)
{
    return {command.payload.begin(),
            std::find(command.payload.begin(), command.payload.end(), 0U)};
}

int main()
{
    Queue queue;
    Queue::Command empty;

    // USB/DMA fragmentation and CRLF must not split or duplicate radio commands.
    queue.append("nrf");
    queue.append("send R 0 0");
    assert(!queue.pop(empty));
    queue.append(" 0 61.5\r\nstatus\n");
    const auto neutral = take(queue, Queue::Kind::transmit);
    assert(text(neutral) == "R 0 0 0 61.5");
    assert(neutral.payload.back() == 0U);
    take(queue, Queue::Kind::status);
    assert(!queue.pop(empty));

    // Serial assistants without line endings use the explicit idle boundary.
    queue.append("nrfsend nrfshow -mr 0");
    queue.finish();
    assert(text(take(queue, Queue::Kind::transmit)) == "nrfshow -mr 0");

    queue.append("joystick off\rR 0 0 0 44.5\njoystick on\nhelp\n");
    take(queue, Queue::Kind::joystick_off);
    assert(text(take(queue, Queue::Kind::transmit)) == "R 0 0 0 44.5");
    take(queue, Queue::Kind::joystick_on);
    take(queue, Queue::Kind::help);

    // The nrfsend prefix does not consume any of the 32 radio payload bytes.
    const std::string full_payload(32U, 'X');
    queue.append("nrfsend " + full_payload + "\n");
    assert(text(take(queue, Queue::Kind::transmit)) == full_payload);
    queue.append("nrfsend " + full_payload + "X\nhelp\n");
    take(queue, Queue::Kind::too_long);
    take(queue, Queue::Kind::help);

    // Reject oversized/malformed input entirely; never send a truncated prefix.
    queue.append("nrfsend " + std::string(150U, 'X'));
    queue.append("\nstatus\n");
    take(queue, Queue::Kind::too_long);
    take(queue, Queue::Kind::status);
    queue.append("nrfsend \nunknown\n");
    take(queue, Queue::Kind::invalid);
    take(queue, Queue::Kind::invalid);
    queue.append(std::string_view("nrfsend R\0 0 0 0 44.5\n", 22U));
    take(queue, Queue::Kind::invalid);

    queue.append("nrfsend incom");
    queue.discardPartial();
    queue.append("plete\nhelp\n");
    take(queue, Queue::Kind::invalid);
    take(queue, Queue::Kind::help);

    queue.append(" \t nrfsend\t hello \t\r\n\r\n");
    assert(text(take(queue, Queue::Kind::transmit)) == "hello");
    queue.append("nrfsend hellx\bo\n");
    assert(text(take(queue, Queue::Kind::transmit)) == "hello");
    assert(!queue.pop(empty));

    // Queue saturation is explicit and preserves the first commands in order.
    for (int index = 0; index < 10; ++index) {
        queue.append("nrfsend " + std::to_string(index) + "\n");
    }
    assert(queue.droppedCommands() == 2U);
    for (int index = 0; index < 8; ++index) {
        assert(text(take(queue, Queue::Kind::transmit)) == std::to_string(index));
    }
    assert(!queue.pop(empty));
    queue.append("status\n");
    take(queue, Queue::Kind::status);
    std::cout << "PASS: serial framing, legacy forwarding, limits, recovery, FIFO\n";
}
