/**
 * csvParserG1.hpp
 * 
 * CSV parser for G1 robot data
 */

#ifndef CSV_PARSER_G1_HPP
#define CSV_PARSER_G1_HPP

#include <string>
#include "G1CSVData.hpp"

// Parse G1 robot CSV file and return G1CSVData structure
G1CSVData ParseG1CSV(const std::string& csv_filename);

#endif // CSV_PARSER_G1_HPP
