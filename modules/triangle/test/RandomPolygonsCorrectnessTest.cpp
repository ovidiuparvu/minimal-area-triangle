#include "triangle/BruteForceMinAreaEnclosingTriangleFinder.hpp"
#include "triangle/LinearMinAreaEnclosingTriangleFinder.hpp"
#include "triangle/exception/ExceptionHandler.hpp"
#include "triangle/exception/InvalidInputException.hpp"
#include "triangle/exception/UnexpectedBehaviourException.hpp"
#include "triangle/util/Filesystem.hpp"

#include <ctime>
#include <fstream>
#include <iostream>

using namespace triangle;
using namespace triangle::util;

const std::string ERR_INVALID_USAGE             = "Usage: RandomPolygonsCorrectnessTest <polygons-input-folder>";
const std::string ERR_INVALID_INPUT_FOLDER_PATH = "The provided polygons input folder path is invalid. Please change.";

const std::string ERR_OPEN_INPUT_FILE   = "The provided polygon input file could not be opened: ";

const std::string ERR_DIFFERENT_RESULTS = "The results obtained by the brute-force and linear time minimal enclosing triangle finders differ for the input file: ";

const std::string MSG_PROCESSING_INPUT_FILE_BEGIN   = "Processing the input file ";
const std::string MSG_PROCESSING_INPUT_FILE_END     = "...";

const std::string INPUT_FILES_EXTENSION = ".txt";


// Return the polygon read from the provided input file stream
void readPolygonFromInputFile(std::ifstream &fin, std::vector<cv::Point2f> &polygon) {
    unsigned int nrOfPoints;
    float xCoordinate;
    float yCoordinate;

    // Read the number of points
    fin >> nrOfPoints;

    // Read the polygon points
    for (unsigned int i = 0; i < nrOfPoints; i++) {
        fin >> xCoordinate >> yCoordinate;

        polygon.push_back(
            cv::Point2f(xCoordinate, yCoordinate)
        );
    }
}

// Read the polygon from the provided input file
void readPolygonInputFile(const std::string &polygonInputFile, std::vector<cv::Point2f> &polygon) {
    std::ifstream fin(polygonInputFile);

    if (fin.is_open()) {
        readPolygonFromInputFile(fin, polygon);
    } else {
        MAT_throw(
            InvalidInputException,
            ERR_OPEN_INPUT_FILE +
            polygonInputFile
        );
    }

    fin.close();
}

// Read the polygon from the provided input file
std::vector<cv::Point2f> readPolygonInputFile(const std::string &polygonInputFile) {
    std::vector<cv::Point2f> polygon;

    // Inform the user which file is processed next
    std::cout << MSG_PROCESSING_INPUT_FILE_BEGIN << polygonInputFile << MSG_PROCESSING_INPUT_FILE_END << std::endl;

    readPolygonInputFile(polygonInputFile, polygon);

    return polygon;
}

// Check if the results of the brute-force and linear minimal enclosing triangle finders are identical
// for the given polygon
bool areIdenticalEnclosingTriangleFindersResults(const std::vector<cv::Point2f> &polygon) {
    std::vector<cv::Point2f> bruteForceEnclosingTriangle;
    std::vector<cv::Point2f> linearEnclosingTriangle;

    // Find the enclosing triangles
    double bruteForceEnclosingTriangleArea  = BruteForceMinAreaEnclosingTriangleFinder().find(
                                                  polygon, bruteForceEnclosingTriangle
                                              );
    double linearEnclosingTriangleArea = LinearMinAreaEnclosingTriangleFinder().find(
                                             polygon, linearEnclosingTriangle
                                         );

    // Check if the computed enclosing triangles are identical
    return (
        Numeric::almostEqual(bruteForceEnclosingTriangleArea, linearEnclosingTriangleArea)
    );
}

// Check if the results of the enclosing triangle finders are identical for the given input files
void validateEnclosingTriangleFindersAgainstFiles(const std::vector<std::string> &polygonsInputFiles) {
    for (auto polygonInputFile : polygonsInputFiles) {
        std::vector<cv::Point2f> polygon = readPolygonInputFile(polygonInputFile);

        if (!areIdenticalEnclosingTriangleFindersResults(polygon)) {
            MAT_throw(
                UnexpectedBehaviourException,
                ERR_DIFFERENT_RESULTS + polygonInputFile
            );
        }
    }
}

// Run the correctness test considering the given polygons input folder path
void runTest(const std::string &polygonsInputFolderPath) {
    if (Filesystem::isValidFolderPath(polygonsInputFolderPath)) {
        std::vector<std::string> polygonsInputFiles = Filesystem::getFilesInFolder(polygonsInputFolderPath,
                                                                                   INPUT_FILES_EXTENSION);

        validateEnclosingTriangleFindersAgainstFiles(polygonsInputFiles);
    } else {
        MAT_throw(InvalidInputException, ERR_INVALID_INPUT_FOLDER_PATH);
    }
}

// Run the random polygons correctness test considering the given polygons input folder path
int runRandomPolygonsCorrectnessTest(const std::string &polygonsInputFolderPath) {
    try {
        runTest(polygonsInputFolderPath);

        return EXEC_SUCCESS_CODE;
    } catch (const MinimalAreaTriangleException &ex) {
        ExceptionHandler::printErrorMessage(ex);

        return EXEC_ERR_CODE;
    }
}

// Main function
int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << ERR_INVALID_USAGE << std::endl;

        return EXEC_ERR_CODE;
    }

    return runRandomPolygonsCorrectnessTest(argv[1]);
}
