use clap::Parser;
use indicatif::ProgressBar;
use geo::{coord, Coord, BoundingRect, Area};
use geo::coordinate_position::{CoordinatePosition, CoordPos};
use ndarray::{Array2, Axis, s};
use std::io::Write;
use std::fs::File;
use std::time::Instant;
use tobj;

//-- CLI parser
#[derive(Parser)]
#[command(author, version, about, long_about = None, allow_negative_numbers(true))]
struct Cli {
    #[arg(short, long)]
    terrain_input: String,
    #[arg(short, long)]
    building_input: String,
    #[arg(short, long)]
    output: String,
    #[arg()]
    cellsize: f64,
    #[arg(short, long, default_value_t = -9999.)]
    nodata: f64,
    #[arg(short, long, default_value_t = 0.)]
    x_transform: f64,
    #[arg(short, long, default_value_t = 0.)]
    y_transform: f64
}

//-- Types
type Point3   = [f64; 3];
type Point2   = [f64; 2];
type Face     = [usize; 3];
type Triangle = [Point3; 3];

//-- Primitives
struct Bbox {
    xmin: f64,
    ymin: f64,
    xmax: f64,
    ymax: f64
}

//-- Basic functions
// Add two points
fn add_pts3(avar: Point3, bvar: Point3) -> Point3 {
    assert_eq!(avar.len(), bvar.len(), "Trying to add unequal lengths!");
    avar.iter().zip(bvar.iter()).map(|(&a, &b)| a + b)
        .collect::<Vec<f64>>()
        .try_into()
        .unwrap()
}

// Linear interpolation within a triangle
pub fn interpolate_linear(triangle: &Triangle, pt: &Coord) -> f64 {
    let a0: f64 = geo::Triangle::new(
        *pt,
        coord!{ x: triangle[1][0], y: triangle[1][1] },
        coord!{ x: triangle[2][0], y: triangle[2][1] }
    ).unsigned_area();
    let a1: f64 = geo::Triangle::new(
        *pt,
        coord!{ x: triangle[0][0], y: triangle[0][1] },
        coord!{ x: triangle[2][0], y: triangle[2][1] }
    ).unsigned_area();
    let a2: f64 = geo::Triangle::new(
        *pt,
        coord!{ x: triangle[0][0], y: triangle[0][1] },
        coord!{ x: triangle[1][0], y: triangle[1][1] }
    ).unsigned_area();

    let mut total = 0.;
    total += triangle[0][2] * a0;
    total += triangle[1][2] * a1;
    total += triangle[2][2] * a2;
    total / (a0 + a1 + a2)
}

//-- Map of OBJ mesh faces
struct Triangles {
    faces : Vec<Vec<Face>>, // vector of face indices per object
    points: Vec<Point3>,
    bbox  : Bbox            // dataset range
}

impl Triangles {
    //-- Constructor using the information from obj import
    pub fn new(firstpt: Point2) -> Self {
        Triangles {
            faces : Vec::new(),
            points: Vec::new(),
            bbox  : Bbox {
                xmin: firstpt[0],
                ymin: firstpt[1],
                xmax: firstpt[0],
                ymax: firstpt[1]
            }
        }
    }

    //-- Methods
    // Add faces (indices to vertex map self.points for an object)
    pub fn add_faces(&mut self, faces: Vec<Face>) {
        self.faces.push(faces);
    }

    // Get the total number of faces
    pub fn num_faces(&self) -> usize {
        let mut num_faces: usize = 0;
        for obj in self.faces.iter() {
            num_faces += obj.len();
        }
        num_faces
    }

    // Add a vertex
    pub fn add_pt(&mut self, pt: Point3) {
        self.points.push(pt);
        // update bbox
        if pt[0] < self.bbox.xmin {
            self.bbox.xmin = pt[0];
        } else if pt[0] > self.bbox.xmax {
            self.bbox.xmax = pt[0];
        }
        if pt[1] < self.bbox.ymin {
            self.bbox.ymin = pt[1];
        } else if pt[1] > self.bbox.ymax {
            self.bbox.ymax = pt[1];
        }
    }

    // Transform points
    // sets origin of the dataset to [XLL, YLL] of self or other dataset and
    // avoids dealing with origin until the output
    pub fn transform_pts(&mut self, transform_pt: Option<Point2>) {
        let pt_t;
        if let Some(pt2) = transform_pt {
            pt_t = [-pt2[0], -pt2[1], 0.];
        } else {
            pt_t = [-self.bbox.xmin, -self.bbox.ymin, 0.];
        }
        for pt in self.points.iter_mut() {
            *pt = add_pts3(*pt, pt_t);
        }
    }

    // Return triangle vertices
    // returns 3x3 array [x, y, z] for every face vertex
    pub fn get_triangle(&self, modelidx: usize, faceidx: usize) -> Triangle {
        assert_eq!(self.faces[modelidx][faceidx].len(), 3,
                   "Triangle structure has more than 3 vertices!");
        [
            self.points[self.faces[modelidx][faceidx][0]],
            self.points[self.faces[modelidx][faceidx][1]],
            self.points[self.faces[modelidx][faceidx][2]]
        ]
    }
    // Return triangle vertices, 2D (x-y) projection in the triangle
    // data struct of package 'geo'
    pub fn get_triangle_geo(&self, modelidx: usize, faceidx: usize) -> geo::Triangle {
        let pt0 = [
            self.points[self.faces[modelidx][faceidx][0]][0],
            self.points[self.faces[modelidx][faceidx][0]][1]
        ];
        let pt1 = [
            self.points[self.faces[modelidx][faceidx][1]][0],
            self.points[self.faces[modelidx][faceidx][1]][1]
        ];
        let pt2 = [
            self.points[self.faces[modelidx][faceidx][2]][0],
            self.points[self.faces[modelidx][faceidx][2]][1]
        ];
        geo::Triangle::new(
            coord!{ x: pt0[0], y: pt0[1] },
            coord!{ x: pt1[0], y: pt1[1] },
            coord!{ x: pt2[0], y: pt2[1] }
        )
    }
}

//-- Load OBJ into vector of triangles
//todo see what to do about the double precision here
fn load_obj(filename: &str, transform_pt: Option<Point2>) -> Triangles {
    println!("Loading file '{}'", filename);
    let load_options = &tobj::LoadOptions {
        triangulate: true,
        ..Default::default()
    };

    let (models, _materials) = tobj::load_obj(filename, load_options)
        .expect("Failed to load OBJ file");
    let firstpt = &models[0].mesh.positions;
    let mut triangles = Triangles::new([firstpt[0] as f64, firstpt[1] as f64]);

    let mut ptstart: usize = 0;
    for (_i, m) in models.iter().enumerate() {
        let mut object_triangles: Vec<Face> = Vec::new();
        let mesh = &m.mesh;
        assert_eq!(mesh.indices.len() % 3, 0, "Faces not triangulated!");
        for fidx in 0..mesh.indices.len() / 3 {
//            println!(" face[{}].indices          = {:?}", face, face_indices);
            let face_indices: Face = [
                mesh.indices[3 * fidx] as usize + ptstart,
                mesh.indices[3 * fidx + 1] as usize + ptstart,
                mesh.indices[3 * fidx + 2] as usize + ptstart,
            ];
            object_triangles.push(face_indices);
        }
        assert_eq!(mesh.positions.len() % 3, 0, "More than 3 points per face!");
        for vtx in 0..mesh.positions.len() / 3 {
            let point = [
                mesh.positions[3 * vtx] as f64,
                mesh.positions[3 * vtx + 1] as f64,
                mesh.positions[3 * vtx + 2] as f64
            ];
            triangles.add_pt(point);
        }
        triangles.add_faces(object_triangles);
        ptstart = triangles.points.len();
    }
    // Transform the coordinate system
    // set it with the optional point which is [XLL, YLL] of another dataset
    // or if no option added, origin is the [XLL, YLL] of the dataset
    triangles.transform_pts(transform_pt);

    return triangles;
}

//-- Raster data structure
#[derive(Clone)]
struct Raster {
    nrows     : usize,
    ncols     : usize,
    cellsize  : f64,
    origin    : Point2,
    nodataval : f64,
    array     : Array2::<f64>
}

impl Raster {
    //-- Constructor
    pub fn new(dataset_range: &Bbox, cellsize: f64, nodata: f64) -> Self {
        let nrows = ((dataset_range.ymax - dataset_range.ymin) / cellsize).abs().ceil() as usize;
        let ncols = ((dataset_range.xmax - dataset_range.xmin) / cellsize).abs().ceil() as usize;
        Raster {
            nrows,
            ncols,
            cellsize,
            origin    : [dataset_range.xmin, dataset_range.ymin],
            nodataval : nodata,
            array     : Array2::from_elem((nrows, ncols),
                                          nodata)
        }
    }

    //-- Methods
    // Get cell centroid coordinates (x-y) in coord data structure
    // of 'geo' package
    pub fn xy_coord_geo(&self, col: usize, row: usize) -> Coord {
        assert!(row < self.nrows, "Invalid row index!");
        assert!(col < self.ncols, "Invalid col index!");
        coord! {
            x: self.cellsize * (0.5 + col as f64),
            y: self.cellsize * (0.5 + row as f64)
        }
    }

    // Set cell value
    pub fn set_val(&mut self, col: usize, row: usize, val: f64) {
        assert!(row < self.nrows, "Invalid row index!");
        assert!(col < self.ncols, "Invalid col index!");
        self.array[[(self.nrows - 1 - row), col]] = val;
    }

    // Return cell value
    pub fn at(&self, col: usize, row: usize) -> &f64 {
        assert!(row < self.nrows, "Invalid row index!");
        assert!(col < self.ncols, "Invalid col index!");
        &self.array[[(self.nrows - 1 - row), col]]
    }

    // Set the XLL and YLL to user-defined coordinates
    pub fn set_output_origin(&mut self, transform_pt: Point2) {
        self.origin[0] += transform_pt[0];
        self.origin[1] += transform_pt[1];
    }

    // Rasterize triangulated faces
    pub fn rasterize(&mut self, triangles: &Triangles,
                     obj_map_opt: Option<&mut Vec<Vec<(usize, usize)>>>) {
        let mut elems: usize = 0;
        for obj in triangles.faces.iter() { elems += obj.len() };
        let pb = ProgressBar::new(elems as u64);

        let mut obj_map_opt = obj_map_opt;
        for (modelidx, model) in triangles.faces.iter().enumerate() {
            // connectivity for one object (model)
            let mut obj_conn: Vec<(usize, usize)> = Vec::new();
                for faceidx in 0..model.len() {
                    let triangle = triangles.get_triangle_geo(modelidx, faceidx);
                    // Get candidate cells from triangle bbox
                    let tri_bbox = triangle.bounding_rect();
                    let colstart = (tri_bbox.min().x.abs() / self.cellsize).floor() as usize;
                    let colend   = (tri_bbox.max().x.abs() / self.cellsize).ceil()  as usize;
                    let rowstart = (tri_bbox.min().y.abs() / self.cellsize).floor() as usize;
                    let rowend   = (tri_bbox.max().y.abs() / self.cellsize).ceil()  as usize;
//        println!("rowstart - rowend: {} - {}", rowstart, rowend);
//        println!("colstart - colend: {} - {}", colstart, colend);

                    // Check candidate cells
                    for i in colstart..colend {
                        for j in rowstart..rowend {
                            let pt = &self.xy_coord_geo(i, j);
                            let coordpos = triangle.coordinate_position(pt);
                            if (coordpos == CoordPos::Inside) || (coordpos == CoordPos::OnBoundary) {
                                // interpolate
                                let height = interpolate_linear(
                                    &triangles.get_triangle(modelidx, faceidx),
                                    pt
                                );
//                                println!("interpolated height: {} at [{}, {}]", height, i, j);
                                // assign if the highest value
                                if height > *self.at(i, j) {
                                    self.set_val(i, j, height);
                                    // update the object-pixel connectivity map
                                    if obj_map_opt.is_some() {
                                        obj_conn.push((i, j));
                                    }
                                }
                            }
                        }
                    }
                    pb.inc(1);
                }
            // add collected map for an object
            if let Some(obj_map) = obj_map_opt.as_mut() {
                if !obj_conn.is_empty() {
                    obj_map.push(obj_conn);
                }
            }
        }
        pb.finish_with_message("done");
    }

    // Trim extra nodatavals
    pub fn trim_raster(&mut self, trim_box: &Bbox) {
        // trim columns from left
        let col_left = ((trim_box.xmin - self.origin[0]).abs() / self.cellsize as f64).floor() as usize;
        // trim columns from right
        let col_right = ((trim_box.xmax - self.origin[0]).abs() / self.cellsize as f64).ceil() as usize;
        // trim rows at the bottom
        let row_bottom = self.nrows
            - ((trim_box.ymin - self.origin[1]).abs() / self.cellsize as f64).floor() as usize;
        // trim rows at the top
        let row_top = self.nrows
            - ((trim_box.ymax - self.origin[1]).abs() / self.cellsize as f64).ceil() as usize;

        self.origin[0] += self.cellsize * col_left as f64;
        self.origin[1] += self.cellsize * (self.array.nrows() - row_bottom) as f64;
        self.array = self.array.slice(s![row_top..row_bottom, col_left..col_right]).to_owned();
        self.nrows = self.array.nrows();
        self.ncols = self.array.ncols();
    }

    // Write raster to disk in ESRI ASC format
    pub fn write_asc(&self, path: String) -> std::io::Result<()> {
        let mut f = File::create(path)?;
        let mut s = String::new();
        // write header
        s.push_str(&format!("NCOLS {}\n",        self.ncols));
        s.push_str(&format!("NROWS {}\n",        self.nrows));
        s.push_str(&format!("XLLCORNER {}\n",    self.origin[0]));
        s.push_str(&format!("YLLCORNER {}\n",    self.origin[1]));
        s.push_str(&format!("CELLSIZE  {}\n",    self.cellsize));
        s.push_str(&format!("NODATA_VALUE {}\n", self.nodataval));
        // write raster data
        for i in 0..self.array.dim().0 {
            let col = self.array.index_axis(Axis(0), i).iter()
                .map(|val| format!("{}", val))
                .collect::<Vec<String>>()
                .join(" ");
            s.push_str(&format!("{}{}", &col, "\n"));
        }
        // output to file
        write!(f, "{}", s).unwrap();
        Ok(())
    }
}

fn main() {
    let start = Instant::now();
    println!("=== CITY RUSTERIZER ===");

    // Grab input agruments
    let cli = Cli::parse();
    let (input_terrain, input_buildings, output, cellsize, nodata)
        = (cli.terrain_input, cli.building_input, String::from(cli.output), cli.cellsize, cli.nodata);
    let transform_pt: Point2 = [cli.x_transform, cli.y_transform];

    // Debug hardcoded data
//    let input_terrain   = "terrain.obj";
//    let input_buildings = "buildings.obj";
//    let output = String::from("output.asc");
//    let cellsize= 0.5;
//    let nodata : f64 = -9999.;
//    let transform_pt: Point2 = [0., 0.];

    // Sort the output names
    if !output.ends_with(".asc") { panic!("Missing .asc file format in the output filename!"); }
    let output_name         = output.trim_end_matches(".asc");
    let output_terrain      = output_name.to_owned() + "_terrain.asc";
    let output_buildings    = output_name.to_owned() + "_buildings.asc";
    let output_buildheights = output_name.to_owned() + "_building_heights.asc";

    // Load objs using the terrain origin for buildings too
    let triangles_terrain   = load_obj(&input_terrain, None);
    let local_orig          = [triangles_terrain.bbox.xmin, triangles_terrain.bbox.ymin];
    let triangles_buildings = load_obj(&input_buildings, Some(local_orig));

    // Initialize rasters
    // both rasters are bound by the terrain bbox to ensure 1-1 mapping
    let mut raster_terrain   = Raster::new(&triangles_terrain.bbox, cellsize, nodata);
    let mut raster_buildings = Raster::new(&triangles_terrain.bbox, cellsize, nodata);

    // Print basic info
    println!("Creating a raster of size: [{}, {}]", raster_terrain.nrows, raster_terrain.ncols);
    println!("Bbox min: [{}, {}]", triangles_terrain.bbox.xmin, triangles_terrain.bbox.ymin);
    println!("Bbox max: [{}, {}]", triangles_terrain.bbox.xmax, triangles_terrain.bbox.ymax);
    println!("Number of faces in terrain:   {:?}", triangles_terrain.num_faces());
    println!("Number of faces in buildings: {:?}", triangles_buildings.num_faces());
    println!("Number of building objects: {:?}", triangles_buildings.faces.len());

    // Loop over triangulated faces and rasterize them
    println!("\nRasterizing terrain...");
    raster_terrain.rasterize(&triangles_terrain, None);

    println!("\nRasterizing buildings...");
    let mut obj_map: Vec<Vec<(usize, usize)>> = Vec::new(); // object-pixel connectivity map
    raster_buildings.rasterize(&triangles_buildings, Some(&mut obj_map));

    // Create the building height raster from the two rasters
    println!("\nCreating building height raster...");
    let mut raster_buildheights = raster_buildings.clone();
    // loop over all building objects
    for obj in obj_map {
        let mut elevations: Vec<f64> = Vec::new();
        // loop over grid cells belonging to a building object
        for cell in &obj {
            // grab elevations from terrain that correspond to building pixels
            elevations.push(*raster_terrain.at(cell.0, cell.1));
        }
        // sort and get lower 10 percentile as terrain height
        elevations.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());
        let height = elevations[(elevations.len() as f64 * 0.1) as usize];
        // loop over building pixels again and calculate height
        for cell in &obj {
            raster_buildheights.set_val(cell.0, cell.1,
                                        raster_buildings.at(cell.0, cell.1) - height);
        }
    }
    // Trim unnecessary nodatavals from building rasters
    raster_buildings.trim_raster(&triangles_buildings.bbox);
    raster_buildheights.trim_raster(&triangles_buildings.bbox);

    // Transform points to the output CRS before writing to disk
    raster_terrain.set_output_origin(transform_pt);
    raster_buildings.set_output_origin(transform_pt);
    raster_buildheights.set_output_origin(transform_pt);

    // Output rasters
    println!("\nWriting rasters to disk...");
    let re = raster_terrain.write_asc(output_terrain.to_string());
    match re {
        Ok(_x) => println!("--> terrain .asc output saved to '{}'", output_terrain),
        Err(_x) => println!("ERROR: path '{}' doesn't exist, abort.", output_buildings),
    }
    let re = raster_buildings.write_asc(output_buildings.to_string());
    match re {
        Ok(_x) => println!("--> building elevations .asc output saved to '{}'", output_buildings),
        Err(_x) => println!("ERROR: path '{}' doesn't exist, abort.", output_buildings),
    }
    let re = raster_buildheights.write_asc(output_buildheights.to_string());
    match re {
        Ok(_x) => println!("--> building heights .asc output saved to '{}'", output_buildheights),
        Err(_x) => println!("ERROR: path '{}' doesn't exist, abort.", output_buildheights),
    }
//    println!("Array: {:?}", raster.array);
    let duration = start.elapsed();
    println!("\nExecution time: {:?}", duration);
    println!("End");
}