# Load the required libraries
library(ggplot2)


# Function for plotting a polygon and its minimal area enclosing triangle 
plotPolygonAndMinimalAreaEnclosingTriangle <- function(polygonInputFile, triangleInputFile, outputFile) {
  # Initialise constants
  POLYGON_COLOUR <- "#1a9641";
  TRIANGLE_COLOUR <- "#d7191c";
  
  TRIANGLE_LINE_WIDTH <- 5;
  POLYGON_POINT_WIDTH <- 5;
  
  # Read the polygon data frame
  polygonDataFrame <<- read.csv(polygonInputFile, sep = " ");
  
  # Read the triangle data frame
  triangleDataFrame <<- read.csv(triangleInputFile, sep = " ");
  
  # Create the plot
  plot <- ggplot() + theme_bw(base_size = 32);
  
  # Add the triangle to the plot
  plot <- plot + geom_polygon(aes(triangleDataFrame$x, triangleDataFrame$y), color = TRIANGLE_COLOUR, fill = NA, size = TRIANGLE_LINE_WIDTH);
  plot <- plot + geom_point(aes(triangleDataFrame$x, triangleDataFrame$y), color = TRIANGLE_COLOUR, fill = NA, size = TRIANGLE_LINE_WIDTH * 2);
  plot <- plot + geom_point(aes(triangleDataFrame$x, triangleDataFrame$y), color = "#ffffff", fill = NA, size = TRIANGLE_LINE_WIDTH * (5 / 4));
  
  # Add the polygon to the plot
  plot <- plot + geom_point(aes(polygonDataFrame$x, polygonDataFrame$y), color = POLYGON_COLOUR, fill = NA, size = POLYGON_POINT_WIDTH);
  plot <- plot + geom_polygon(aes(polygonDataFrame$x, polygonDataFrame$y), color = POLYGON_COLOUR, fill = NA, size = (POLYGON_POINT_WIDTH / 4));
  
  # Set the relevant settings for the plot
  plot <- plot + xlab("x") + ylab("y");
  
  # Output the plot as a svg image
  svg(outputFile, width = 9.6, height = 7.2);
  
  print(plot);
  
  graphics.off();
}