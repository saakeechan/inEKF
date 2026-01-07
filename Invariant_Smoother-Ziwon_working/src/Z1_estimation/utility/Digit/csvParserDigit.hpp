#ifndef CSV_PARSER_DIGIT_HPP
#define CSV_PARSER_DIGIT_HPP

#include "DigitCSVData.hpp"
#include <string>

/**
 * Parse CSV file containing Digit robot sensor data
 * @param csv_filename Path to the CSV file
 * @return DigitCSVData structure containing parsed data
 */
DigitCSVData ParseDigitCSV(const std::string& csv_filename);

#endif // CSV_PARSER_DIGIT_HPP
