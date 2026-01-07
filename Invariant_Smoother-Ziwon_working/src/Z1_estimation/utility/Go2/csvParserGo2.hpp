/**
 * csvParserGo2.hpp
 * 
 * CSV parser for Go2 robot data
 */

#ifndef CSV_PARSER_Go2_HPP
#define CSV_PARSER_Go2_HPP

#include <string>
#include "Go2CSVData.hpp"

// Parse Go2 robot CSV file and return Go2CSVData structure
Go2CSVData ParseGo2CSV(const std::string& csv_filename);

#endif // CSV_PARSER_Go2_HPP
