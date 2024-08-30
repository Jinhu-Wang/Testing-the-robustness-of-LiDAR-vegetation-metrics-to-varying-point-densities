
library(sf)
library(ggplot2)
library(rgdal)
library(gstat)
library(eSDM)

workingdirectory="Path\to\Your\Working\Directory"

setwd(workingdirectory)

shp <- readOGR(dsn=".",layer="the_shpfiles_of_centroids_of_100_randomly_placed_plot_in_Grassland")

centroids <- st_as_sf(shp)

str(centroids)

# set the half_plot_length for the plots Options:[0.5, 1, 2.5, 5, 10, 15]
half_plot_length <- 15 # half_plot_length in meters


# define the plot edges based upon the plot half_plot_length 
yPlus <- centroids$POINT_Y+half_plot_length
xPlus <- centroids$POINT_X+half_plot_length
yMinus <- centroids$POINT_Y-half_plot_length
xMinus <- centroids$POINT_X-half_plot_length

# calculate polygon coordinates for each plot centroid. 
square=cbind(xMinus,yPlus,  # NW corner
             xPlus, yPlus,  # NE corner
             xPlus,yMinus,  # SE corner
             xMinus,yMinus, # SW corner
             xMinus,yPlus)  # NW corner again - close ploygon

# Extract the plot ID information
ID=centroids$Plot_ID

# create spatial polygons from coordinates
polys <- SpatialPolygons(mapply(function(poly, id) {
  xy <- matrix(poly, ncol=2, byrow=TRUE)
  Polygons(list(Polygon(xy)), ID=id)
}, 
split(square, row(square)), ID),
proj4string=CRS(as.character("+proj=sterea +lat_0=52.15616055555555 +lon_0=5.38763888888889 +k=0.9999079 +x_0=155000 +y_0=463000 +ellps=bessel +units=m +no_defs")))

# create SpatialPolygonDataFrame -- this step is required to output multiple polygons.
polys.df <- SpatialPolygonsDataFrame(polys, data.frame(id=ID, row.names=ID))

# write the shapefiles 
writeOGR(polys.df, '.', 'Plot_grassland_30m', 'ESRI Shapefile')

