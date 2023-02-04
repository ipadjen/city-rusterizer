# city-rusterizer

Convert 3D geometries of cities to a 2D raster.

### Data formats
Input: terrain and buildings in Wavefront OBJ format

Output: terrain elevation, building elevation, and building height raster in [Esri ASCII](https://en.wikipedia.org/wiki/Esri_grid) or [GeoTIFF](https://en.wikipedia.org/wiki/GeoTIFF) format

### Compilation

Clone the repository and build with

```
cargo build --release
```

Optionally, to export to GeoTIFF format, you have to install GDAL and compile using 'with_gdal' feature
```
cargo build --release --features with_gdal
```

Access the executable by typing

```
./target/release/city-rusterizer
```

You can get Cargo [here](https://www.rust-lang.org/tools/install).

### Usage
```
city-rusterizer -b <buildings_input> -t <terrain_input> -o <output> <cellsize>
``` 
e.g.
```
city-rusterizer -b buildings.obj -t terrain.obj -o output.asc 0.5
```

You can store CRS information if using GeoTIFF by adding ```--epsg <EPSG>``` flag.
