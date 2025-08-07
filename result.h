#pragma once
#include <variant>
#include <string>

template<typename T>
class Result {
public:
    static Result success(T value) {
        return Result(std::move(value));
    }
    
    static Result error(std::string error) {
        return Result(std::move(error));
    }
    
    bool isSuccess() const { return std::holds_alternative<T>(data); }
    bool isError() const { return std::holds_alternative<std::string>(data); }
    
    const T& value() const { return std::get<T>(data); }
    const std::string& error() const { return std::get<std::string>(data); }
    
private:
    explicit Result(T value) : data(std::move(value)) {}
    explicit Result(std::string error) : data(std::move(error)) {}
    
    std::variant<T, std::string> data;
};