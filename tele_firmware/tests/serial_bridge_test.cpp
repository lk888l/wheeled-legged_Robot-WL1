#include <cassert>
#include <iostream>
#include <string>
#include "SerialCommandBridge.hpp"
#include "task.h"

using Result = BridgeCommandParser::Result;
static SerialCommandBridge* interrupted_bridge = nullptr;

static Result feed(BridgeCommandParser& parser, std::string_view bytes)
{
    auto result = Result::incomplete;
    for (const unsigned char byte : bytes) { result = parser.feed(byte); }
    return result;
}

static void receive(SerialCommandBridge& bridge, UART_HandleTypeDef& uart, std::string_view bytes)
{
    for (const unsigned char byte : bytes) {
        assert(uart.receiving != nullptr);
        *uart.receiving = byte;
        uart.RxState = HAL_UART_STATE_READY;
        bridge.onReceiveFromIsr();
    }
}

static void test_parser()
{
    BridgeCommandParser parser;
    assert(feed(parser, "angle") == Result::incomplete);
    assert(feed(parser, "pid -p 1.5") == Result::incomplete);
    assert(feed(parser, "\n") == Result::accepted);
    assert(parser.completed() == "anglepid -p 1.5");
    for (const auto command : {"anglebias -2.5\r\n", "velocitypid -d 2e-3\n",
                              "differpid -i .1\n", "rollpid -p 1\n",
                              "anglepid auto\n", "anglebias auto\n"}) {
        assert(feed(parser, command) == Result::accepted);
    }
    const std::string limit = "anglebias " + std::string(21U, '1');
    assert(limit.size() == 31U);
    assert(feed(parser, limit + "\r\n") == Result::accepted);
    assert(feed(parser, limit + "1\n") == Result::rejected);
    for (const auto command : {"R 0 0 0 61.5\n", "legheight 50\n", "target_roll 1\n",
                              "VandD 0 0\n", "showimu -y\n", "nrfshow -mp 0\n",
                              "rollpid -d 1\n", "anglepid -p nan\n", "anglebias inf\n",
                              "anglebias 1oops\n", "anglebias 1 2\n", "anglebias 1e\n",
                              "anglebias 1\r2\n", "anglebias \t1\n"}) {
        assert(feed(parser, command) == Result::rejected);
    }
    assert(feed(parser, std::string("anglebias 1\0", 12U) + "\n") == Result::rejected);
    assert(feed(parser, "anglebias 1\n") == Result::accepted);
    parser.discardLine();
    assert(feed(parser, "anglebias 2\n") == Result::rejected);
    assert(feed(parser, "anglebias 3\n") == Result::accepted);
    assert(feed(parser, "\n\r\n") == Result::incomplete);
    assert(feed(parser, "anglebias 4\n") == Result::accepted);
}

static void test_bridge()
{
    UART_HandleTypeDef uart;
    SerialCommandBridge bridge(&uart);
    LkUart log;
    RemoteCommandCodec::Payload payload{};
    test_tick = 0U;
    assert(bridge.start());
    bridge.service(log, false); // Empty offline poll must not poison first line.
    receive(bridge, uart, "anglepid -p ");
    bridge.service(log);
    assert(!bridge.take(payload));
    receive(bridge, uart, "1\nanglebias -2\n");
    bridge.service(log);
    assert(bridge.take(payload));
    assert(std::string(reinterpret_cast<const char*>(payload.data())) == "anglepid -p 1");
    assert(payload[31] == 0U);
    assert(bridge.take(payload));
    assert(std::string(reinterpret_cast<const char*>(payload.data())) == "anglebias -2");
    assert(!bridge.take(payload));

    for (int i = 0; i < 5; ++i) {
        receive(bridge, uart, "anglebias 1\n");
        bridge.service(log);
    }
    assert(log.messages.back() == "bridge: rejected queue full\n");
    for (int i = 0; i < 4; ++i) { assert(bridge.take(payload)); }
    assert(!bridge.take(payload));

    receive(bridge, uart, "anglebias 1\n");
    bridge.service(log);
    test_tick = 501U;
    bridge.service(log);
    assert(!bridge.take(payload));
    assert(log.messages.back() == "bridge: rejected expired command\n");

    receive(bridge, uart, std::string(160U, '1') + "\n");
    bridge.service(log);
    assert(!bridge.take(payload));
    receive(bridge, uart, "\nanglebias 2\n");
    bridge.service(log);
    assert(bridge.take(payload));
    assert(std::string(reinterpret_cast<const char*>(payload.data())) == "anglebias 2");

    receive(bridge, uart, "anglebias 1\nanglepid ");
    bridge.service(log, false);
    receive(bridge, uart, "-p 1\nanglebias 3\n");
    bridge.service(log);
    assert(bridge.take(payload));
    assert(std::string(reinterpret_cast<const char*>(payload.data())) == "anglebias 3");
    assert(!bridge.take(payload));

    uart.RxState = HAL_UART_STATE_READY;
    bridge.onErrorFromIsr();
    bridge.service(log);
    assert(uart.RxState == HAL_UART_STATE_BUSY_RX);
    receive(bridge, uart, "\nanglebias 4\n");
    bridge.service(log);
    assert(bridge.take(payload));
    assert(std::string(reinterpret_cast<const char*>(payload.data())) == "anglebias 4");

    receive(bridge, uart, "anglebias 5\n");
    interrupted_bridge = &bridge;
    test_hook_countdown = 13; // Snapshot + each byte, error immediately after LF pop.
    test_critical_exit_hook = [] { interrupted_bridge->onErrorFromIsr(); };
    bridge.service(log);
    assert(!bridge.take(payload));
    bridge.service(log);
    assert(!bridge.take(payload));

    // HAL reports a bad byte to RxCplt before invoking ErrorCallback.
    *uart.receiving = '\n';
    uart.RxState = HAL_UART_STATE_READY;
    uart.ErrorCode = 1U;
    bridge.onReceiveFromIsr();
    assert(uart.ErrorCode == 1U); // ISR must not rearm and erase the HAL error.
    bridge.service(log);
    assert(uart.ErrorCode == HAL_UART_ERROR_NONE);
    receive(bridge, uart, "\nanglebias 6\n");
    bridge.service(log);
    assert(bridge.take(payload));
    assert(std::string(reinterpret_cast<const char*>(payload.data())) == "anglebias 6");
}

int main()
{
    test_parser();
    test_bridge();
    std::cout << "serial bridge parser, queue, framing, error and expiry tests passed\n";
}
