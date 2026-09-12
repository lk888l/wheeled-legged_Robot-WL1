#pragma once
#include <string>
#include <vector>
class LkUart {
public:
    std::vector<std::string> messages;
    void print(const char* text) { messages.emplace_back(text); }
    void print(const char* format, const char* value)
    {
        std::string text(format);
        text.replace(text.find("{}"), 2U, value);
        messages.push_back(text);
    }
};
