/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <array>
#include <cassert>
#include <cctype>
#include <fstream>
#include <string>
#include <vector>
#include <cstdint>
#include <algorithm>
#include <utility>
#include <uavcan_linux/exception.hpp>

namespace uavcan_linux
{
/**
 * This class can find and read machine ID from a text file, represented as 32-char (16-byte) long hexadecimal string,
 * possibly with separators (like dashes or colons). If the available ID is more than 16 bytes, extra bytes will be
 * ignored. A shorter ID will not be accepted as valid.
 * In order to be read, the ID must be located on the first line of the file and must not contain any whitespace
 * characters.
 *
 * Examples of valid ID:
 *   0123456789abcdef0123456789abcdef
 *   20CE0b1E-8C03-07C8-13EC-00242C491652
 */
class MachineIDReader
{
public:
    static constexpr int MachineIDSize = 16;

    typedef std::array<std::uint8_t, MachineIDSize> MachineID;

    static std::vector<std::string> getDefaultSearchLocations()
    {
        return
        {
            "/etc/machine-id",
            "/var/lib/dbus/machine-id",
            "/sys/class/dmi/id/product_uuid"
        };
    }

private:
    const std::vector<std::string> search_locations_;

    static std::vector<std::string> mergeLists(const std::vector<std::string>& a, const std::vector<std::string>& b)
    {
        std::vector<std::string> ab;
        ab.reserve(a.size() + b.size());
        ab.insert(ab.end(), a.begin(), a.end());
        ab.insert(ab.end(), b.begin(), b.end());
        return ab;
    }

    bool tryRead(const std::string& location, MachineID& out_id) const
    {
        /*
         * Reading the file
         */
        std::string token;
        try
        {
            std::ifstream infile(location);
            infile >> token;
        }
        catch (std::exception&)
        {
            return false;
        }

        /*
         * Preprocessing the input string - convert to lowercase, remove all non-hex characters, limit to 32 chars
         */
        std::transform(token.begin(), token.end(), token.begin(), [](char x) { return std::tolower(x); });
        token.erase(std::remove_if(token.begin(), token.end(),
                                   [](char x){ return (x < 'a' || x > 'f') && !std::isdigit(x); }),
                    token.end());

        if (token.length() < (MachineIDSize * 2))
        {
            return false;
        }
        token.resize(MachineIDSize * 2);        // Truncating

        /*
         * Parsing the string as hex bytes
         */
        auto sym = std::begin(token);
        for (auto& byte : out_id)
        {
            assert(sym != std::end(token));
            byte = std::stoi(std::string{*sym++, *sym++}, nullptr, 16);
        }

        return true;
    }

public:
    /**
     * This class can use extra seach locations. If provided, they will be checked first, before default ones.
     */
    MachineIDReader(const std::vector<std::string>& extra_search_locations = {})
        : search_locations_(mergeLists(extra_search_locations, getDefaultSearchLocations()))
    { }

    /**
     * Just like @ref readAndGetLocation(), but this one doesn't return location where this ID was obtained from.
     */
    MachineID read() const { return readAndGetLocation().first; }

    /**
     * This function checks available search locations and reads the ID from the first valid location.
     * It returns std::pair<> with ID and the file path where it was read from.
     * In case if none of the search locations turned out to be valid, @ref uavcan_linux::Exception will be thrown.
     */
    std::pair<MachineID, std::string> readAndGetLocation() const
    {
        for (auto x : search_locations_)
        {
            auto out = MachineID();
            if (tryRead(x, out))
            {
                return {out, x};
            }
        }
        throw Exception("Failed to read machine ID");
    }
};


}
