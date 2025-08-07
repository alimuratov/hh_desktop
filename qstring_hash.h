#pragma once
#include <QString>
#include <QHash>

// Hash functor for QString to use with std::unordered_map
struct QStringHash {
    std::size_t operator()(const QString &s) const noexcept { 
        return qHash(s); 
    }
};
