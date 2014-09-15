#!/bin/bash

###############################################################################
#
# This script takes a benchmark input file as input (i.e. list of polygon 
# points coordinates) and returns a corresponding C++ definition of the polygon
# represented as a std::vector.
#
###############################################################################

# Check if an input parameter was specified
if [[ $# -ne 1 ]];
then
    echo "Usage: CreateC++PolygonRepresentation.sh <benchmark-input-file>";

    exit 1;
fi

# Check if the input parameter is a regular file
if [[ ! -f $1 ]];
then
    echo "An invalid file was provided as input. Please change.";

    exit 1;
fi

# Set the internal field separator
IFS=''; 

# Initialise the string containing the C++ polygon representation
cppPolygonRepresentation="std::vector<cv::Point2f>({"; 

# Store the contents of the input file in a variable
lines=$(cat $1 | tail -n+2); 

# Parse the input file
while read line; 
do 
    xCoordinate=$(echo ${line} | cut -f1 -d " "); 
    yCoordinate=$(echo ${line} | cut -f2 -d " "); 

    cppPolygonRepresentation=${cppPolygonRepresentation}"cv::Point2f(${xCoordinate}, ${yCoordinate}), "; 
done <<< ${lines}; 

# Set the suffix of the string containing the C++ polygon representation
cppPolygonRepresentation=${cppPolygonRepresentation%??}"});";

# Print out the resulting C++ polygon representation
echo ${cppPolygonRepresentation};
