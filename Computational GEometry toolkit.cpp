#include <iostream>
#include <vector>
#include <algorithm> 
#include <random>    
#include <chrono>    
#include <limits>    

// CGAL Headers for 2D Geometry
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h> // Though often derived from Delaunay
#include <CGAL/Min_circle_2.h>       // For Smallest Enclosing Circle
#include <CGAL/Polygon_2.h>          // For Polygon operations
#include <CGAL/intersections.h>      // For segment/line intersections
#include <CGAL/Bbox_2.h>             // For Axis-Aligned Bounding Box

// Define the kernel.
// Exact_predicates_inexact_constructions_kernel is a good balance for many applications,
// providing robust predicates (e.g., orientation, in_circle) while using floating point
// arithmetic for constructions (e.g., circumcenter, intersection points).
// This choice ensures geometric decisions are correct, even if coordinates of constructed
// objects might have small floating-point inaccuracies.
// For fully exact constructions, you would use CGAL::Exact_predicates_exact_constructions_kernel.
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

// Define common geometric types from the chosen kernel
typedef K::Point_2 Point;
typedef K::Segment_2 Segment;
typedef K::Triangle_2 Triangle;
typedef K::Line_2 Line;
typedef K::Vector_2 Vector;
typedef K::Iso_rectangle_2 Iso_rectangle; // For Axis-Aligned Bounding Box

// Define the Delaunay triangulation type
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;

// Define the Min_circle type
typedef CGAL::Min_circle_2<K> Min_circle;

// Define the Polygon type
typedef CGAL::Polygon_2<K> Polygon;

// Helper function to print a point
void print_point(const Point& p, const std::string& prefix = "") {
    std::cout << prefix << "(" << p.x() << ", " << p.y() << ")";
}

// Main function where the computational geometry toolkit demonstration runs
int main() {
    std::cout << "--- Computational Geometry Toolkit Demonstration (CGAL) ---" << std::endl;
    std::cout << "This example uses CGAL to demonstrate various 2D computational geometry concepts:" << std::endl;
    std::cout << "Convex Hull, Delaunay Triangulation, Voronoi Diagram, Smallest Enclosing Circle," << std::endl;
    std::cout << "Point-in-Polygon Test, Polygon Area/Orientation, Segment Intersection, Closest Pair," << std::endl;
    std::cout << "and Axis-Aligned Bounding Box (AABB)." << std::endl;
    std::cout << "---------------------------------------------------------" << std::endl << std::endl;

    // 1. Generate a set of random points for demonstration
    std::vector<Point> points;
    int num_points = 25; // Increased number of points for more complex outputs
    // Use a high-resolution clock for a more random seed
    std::default_random_engine generator(std::chrono::high_resolution_clock::now().time_since_epoch().count());
    // Define a distribution range for point coordinates
    std::uniform_real_distribution<double> distribution(-150.0, 150.0);

    std::cout << "SECTION 1: Generating Random Points" << std::endl;
    std::cout << "---------------------------------" << std::endl;
    std::cout << "Generating " << num_points << " random points within [-150, 150] x [-150, 150]..." << std::endl;
    for (int i = 0; i < num_points; ++i) {
        points.push_back(Point(distribution(generator), distribution(generator)));
        print_point(points.back(), "  Point " + std::to_string(i + 1) + ": ");
        std::cout << std::endl;
    }
    std::cout << std::endl;

    // 2. Convex Hull Computation
    // The convex hull is the smallest convex polygon that contains all the points in a set.
    // CGAL provides an efficient implementation for this.
    std::vector<Point> hull;
    CGAL::convex_hull_2(points.begin(), points.end(), std::back_inserter(hull));

    std::cout << "SECTION 2: Convex Hull Computation" << std::endl;
    std::cout << "---------------------------------" << std::endl;
    std::cout << "Computed the convex hull of the " << num_points << " points." << std::endl;
    std::cout << "Number of points on the convex hull: " << hull.size() << std::endl;
    std::cout << "Points on the convex hull (in counter-clockwise order):" << std::endl;
    for (size_t i = 0; i < hull.size(); ++i) {
        print_point(hull[i], "  Hull Point " + std::to_string(i + 1) + ": ");
        std::cout << std::endl;
    }
    std::cout << std::endl;

    // 3. Delaunay Triangulation
    // A Delaunay triangulation is a triangulation of a set of points such that no point
    // in the set is inside the circumcircle of any triangle in the triangulation.
    Delaunay dt;
    dt.insert(points.begin(), points.end());

    std::cout << "SECTION 3: Delaunay Triangulation" << std::endl;
    std::cout << "---------------------------------" << std::endl;
    std::cout << "Constructed Delaunay triangulation from " << num_points << " points." << std::endl;
    std::cout << "Number of vertices in Delaunay triangulation: " << dt.number_of_vertices() << std::endl;
    std::cout << "Number of finite faces (triangles): " << dt.number_of_faces() << std::endl;

    // Iterate over finite faces (triangles) and print their vertices
    std::cout << "Finite triangles (vertices listed in counter-clockwise order):" << std::endl;
    int triangle_count = 0;
    for (Delaunay::Face_iterator fit = dt.finite_faces_begin(); fit != dt.finite_faces_end(); ++fit) {
        triangle_count++;
        std::cout << "  Triangle " << triangle_count << ": ";
        for (int i = 0; i < 3; ++i) {
            Point p = fit->vertex(i)->point();
            print_point(p);
            std::cout << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    // 4. Voronoi Diagram (derived from Delaunay Triangulation)
    // The Voronoi diagram is the geometric dual of the Delaunay triangulation.
    // Each vertex of the Delaunay triangulation corresponds to a Voronoi cell.
    // Each edge of the Delaunay triangulation corresponds to a Voronoi edge.
    // Each finite face (triangle) of the Delaunay triangulation corresponds to a Voronoi vertex (circumcenter).
    std::cout << "SECTION 4: Voronoi Diagram (Dual of Delaunay Triangulation)" << std::endl;
    std::cout << "---------------------------------------------------------" << std::endl;

    std::cout << "Voronoi vertices (circumcenters of Delaunay triangles):" << std::endl;
    int voronoi_vertex_count = 0;
    for (Delaunay::Face_iterator fit = dt.finite_faces_begin(); fit != dt.finite_faces_end(); ++fit) {
        voronoi_vertex_count++;
        // The circumcenter of a Delaunay triangle is a vertex of the Voronoi diagram.
        Point voronoi_vertex = dt.circumcenter(fit);
        print_point(voronoi_vertex, "  Voronoi Vertex " + std::to_string(voronoi_vertex_count) + ": ");
        std::cout << " (Circumcenter of Delaunay triangle: ";
        for (int i = 0; i < 3; ++i) {
            print_point(fit->vertex(i)->point());
            std::cout << " ";
        }
        std::cout << ")" << std::endl;
    }
    std::cout << std::endl;

    std::cout << "Voronoi edges (perpendicular bisectors of Delaunay edges):" << std::endl;
    int voronoi_edge_count = 0;
    for (Delaunay::Edge_iterator eit = dt.finite_edges_begin(); eit != dt.finite_edges_end(); ++eit) {
        voronoi_edge_count++;
        // An edge in the Delaunay triangulation corresponds to a Voronoi edge.
        // The Voronoi edge connects the circumcenters of the two adjacent Delaunay faces.
        // If one of the faces is infinite, the Voronoi edge extends to infinity.

        Delaunay::Face_handle f1 = eit->first;
        Delaunay::Face_handle f2 = f1->neighbor(eit->second); // Get the adjacent face

        std::cout << "  Voronoi Edge " << voronoi_edge_count << ": ";
        if (dt.is_infinite(f1) || dt.is_infinite(f2)) {
            // This Delaunay edge is on the convex hull, so the corresponding Voronoi edge
            // extends to infinity. We can't easily print its full extent here without
            // more complex handling of infinite rays.
            std::cout << "Connects finite Voronoi vertex to infinity. (Dual to Delaunay edge between ";
            print_point(f1->vertex((eit->third + 1) % 3)->point());
            std::cout << " and ";
            print_point(f1->vertex((eit->third + 2) % 3)->point());
            std::cout << ")" << std::endl;
        } else {
            Point voronoi_vertex1 = dt.circumcenter(f1);
            Point voronoi_vertex2 = dt.circumcenter(f2);
            std::cout << "From ";
            print_point(voronoi_vertex1);
            std::cout << " to ";
            print_point(voronoi_vertex2);
            std::cout << " (Dual to Delaunay edge between ";
            print_point(f1->vertex((eit->third + 1) % 3)->point());
            std::cout << " and ";
            print_point(f1->vertex((eit->third + 2) % 3)->point());
            std::cout << ")" << std::endl;
        }
    }
    std::cout << std::endl;

    // 5. Smallest Enclosing Circle (Min-Disk)
    // Finds the circle of minimum radius that encloses all given points.
    std::cout << "SECTION 5: Smallest Enclosing Circle (Min-Disk)" << std::endl;
    std::cout << "-----------------------------------------------" << std::endl;
    Min_circle mc(points.begin(), points.end());

    std::cout << "Smallest enclosing circle properties:" << std::endl;
    std::cout << "  Center: ";
    print_point(mc.center());
    std::cout << std::endl;
    std::cout << "  Radius: " << std::sqrt(mc.squared_radius()) << std::endl;
    std::cout << "  Number of points on the boundary of the circle: " << mc.number_of_support_points() << std::endl;
    std::cout << "  Support points (points defining the circle):" << std::endl;
    for (typename Min_circle::Support_point_iterator it = mc.support_points_begin(); it != mc.support_points_end(); ++it) {
        print_point(*it, "    ");
        std::cout << std::endl;
    }
    std::cout << std::endl;

    // 6. Point in Polygon Test
    // Determines if a query point is inside, outside, or on the boundary of a given polygon.
    // We'll use the convex hull as our test polygon.
    if (hull.size() < 3) {
        std::cout << "Cannot perform Point in Polygon test: Convex hull has fewer than 3 points." << std::endl;
    } else {
        Polygon poly(hull.begin(), hull.end());
        std::cout << "SECTION 6: Point in Polygon Test" << std::endl;
        std::cout << "------------------------------" << std::endl;
        std::cout << "Using the convex hull as the test polygon." << std::endl;
        std::cout << "Polygon vertices: ";
        for (const auto& p : poly.vertices()) {
            print_point(p);
            std::cout << " ";
        }
        std::cout << std::endl;

        // Test points
        Point test_point1 = poly.vertex(0); // A vertex on the boundary
        Point test_point2 = CGAL::centroid(poly.vertices_begin(), poly.vertices_end()); // Centroid (likely inside)
        Point test_point3(distribution(generator) * 2, distribution(generator) * 2); // Random point (likely outside)
        Point test_point4 = poly.vertex(0) + Vector(1, 1); // Slightly outside from a vertex

        std::vector<Point> test_points = {test_point1, test_point2, test_point3, test_point4};

        for (size_t i = 0; i < test_points.size(); ++i) {
            std::cout << "  Testing point " << i + 1 << ": ";
            print_point(test_points[i]);
            std::cout << " is ";
            CGAL::Bounded_side side = poly.bounded_side(test_points[i]);
            if (side == CGAL::ON_UNBOUNDED_SIDE) {
                std::cout << "outside the polygon." << std::endl;
            } else if (side == CGAL::ON_BOUNDED_SIDE) {
                std::cout << "inside the polygon." << std::endl;
            } else { // CGAL::ON_BOUNDARY
                std::cout << "on the boundary of the polygon." << std::endl;
            }
        }
        std::cout << std::endl;
    }

    // 7. Polygon Area and Orientation
    // Calculates the area of a polygon and determines if its vertices are ordered clockwise or counter-clockwise.
    if (hull.size() < 3) {
        std::cout << "Cannot calculate Polygon Area/Orientation: Convex hull has fewer than 3 points." << std::endl;
    } else {
        Polygon poly_for_area(hull.begin(), hull.end());
        std::cout << "SECTION 7: Polygon Area and Orientation" << std::endl;
        std::cout << "-------------------------------------" << std::endl;
        std::cout << "Using the convex hull as the polygon for area/orientation." << std::endl;

        std::cout << "  Polygon Area: " << poly_for_area.area() << std::endl;

        std::cout << "  Polygon Orientation: ";
        CGAL::Orientation orientation = poly_for_area.orientation();
        if (orientation == CGAL::CLOCKWISE) {
            std::cout << "CLOCKWISE" << std::endl;
        } else if (orientation == CGAL::COUNTERCLOCKWISE) {
            std::cout << "COUNTERCLOCKWISE" << std::endl;
        } else { // CGAL::COLLINEAR
            std::cout << "COLLINEAR (degenerate polygon)" << std::endl;
        }
        std::cout << std::endl;
    }

    // 8. Segment Intersection
    // Determines if two line segments intersect and, if so, finds the intersection point(s).
    std::cout << "SECTION 8: Segment Intersection" << std::endl;
    std::cout << "-----------------------------" << std::endl;

    // Test case 1: Intersecting segments
    Segment s1_intersect(Point(-50, -50), Point(50, 50));
    Segment s2_intersect(Point(-50, 50), Point(50, -50));
    std::cout << "  Test Case 1: Intersecting segments" << std::endl;
    std::cout << "    Segment 1: From "; print_point(s1_intersect.source()); std::cout << " to "; print_point(s1_intersect.target()); std::cout << std::endl;
    std::cout << "    Segment 2: From "; print_point(s2_intersect.source()); std::cout << " to "; print_point(s2_intersect.target()); std::cout << std::endl;
    auto result1 = CGAL::intersection(s1_intersect, s2_intersect);
    if (result1) {
        if (const Point* p = boost::get<Point>(&*result1)) {
            std::cout << "    Intersection is a point: "; print_point(*p); std::cout << std::endl;
        } else if (const Segment* s = boost::get<Segment>(&*result1)) {
            std::cout << "    Intersection is a segment: From "; print_point(s->source()); std::cout << " to "; print_point(s->target()); std::cout << std::endl;
        }
    } else {
        std::cout << "    Segments do not intersect." << std::endl;
    }
    std::cout << std::endl;

    // Test case 2: Non-intersecting segments
    Segment s3_no_intersect(Point(0, 0), Point(10, 10));
    Segment s4_no_intersect(Point(20, 0), Point(30, 10));
    std::cout << "  Test Case 2: Non-intersecting segments" << std::endl;
    std::cout << "    Segment 1: From "; print_point(s3_no_intersect.source()); std::cout << " to "; print_point(s3_no_intersect.target()); std::cout << std::endl;
    std::cout << "    Segment 2: From "; print_point(s4_no_intersect.source()); std::cout << " to "; print_point(s4_no_intersect.target()); std::cout << std::endl;
    auto result2 = CGAL::intersection(s3_no_intersect, s4_no_intersect);
    if (result2) {
        // This block will not be reached for non-intersecting segments
    } else {
        std::cout << "    Segments do not intersect." << std::endl;
    }
    std::cout << std::endl;

    // Test case 3: Collinear and overlapping segments
    Segment s5_collinear_overlap(Point(0, 0), Point(10, 0));
    Segment s6_collinear_overlap(Point(5, 0), Point(15, 0));
    std::cout << "  Test Case 3: Collinear and overlapping segments" << std::endl;
    std::cout << "    Segment 1: From "; print_point(s5_collinear_overlap.source()); std::cout << " to "; print_point(s5_collinear_overlap.target()); std::cout << std::endl;
    std::cout << "    Segment 2: From "; print_point(s6_collinear_overlap.source()); std::cout << " to "; print_point(s6_collinear_overlap.target()); std::cout << std::endl;
    auto result3 = CGAL::intersection(s5_collinear_overlap, s6_collinear_overlap);
    if (result3) {
        if (const Point* p = boost::get<Point>(&*result3)) {
            std::cout << "    Intersection is a point: "; print_point(*p); std::cout << std::endl;
        } else if (const Segment* s = boost::get<Segment>(&*result3)) {
            std::cout << "    Intersection is a segment: From "; print_point(s->source()); std::cout << " to "; print_point(s->target()); std::cout << std::endl;
        }
    } else {
        std::cout << "    Segments do not intersect." << std::endl;
    }
    std::cout << std::endl;

    // 9. Closest Pair of Points
    // Finds the two points in a set that are closest to each other.
    // CGAL provides more advanced structures for this (e.g., k-d trees),
    // but for demonstration, we'll use a simple brute-force approach with CGAL's distance function.
    std::cout << "SECTION 9: Closest Pair of Points" << std::endl;
    std::cout << "-------------------------------" << std::endl;

    if (points.size() < 2) {
        std::cout << "  Need at least two points to find a closest pair." << std::endl;
    } else {
        double min_squared_dist = std::numeric_limits<double>::max();
        Point p1_closest, p2_closest;

        for (size_t i = 0; i < points.size(); ++i) {
            for (size_t j = i + 1; j < points.size(); ++j) {
                double current_squared_dist = CGAL::squared_distance(points[i], points[j]);
                if (current_squared_dist < min_squared_dist) {
                    min_squared_dist = current_squared_dist;
                    p1_closest = points[i];
                    p2_closest = points[j];
                }
            }
        }
        std::cout << "  The closest pair of points is:" << std::endl;
        std::cout << "    Point 1: "; print_point(p1_closest); std::cout << std::endl;
        std::cout << "    Point 2: "; print_point(p2_closest); std::cout << std::endl;
        std::cout << "  Squared distance: " << min_squared_dist << std::endl;
        std::cout << "  Actual distance: " << std::sqrt(min_squared_dist) << std::endl;
    }
    std::cout << std::endl;

    // 10. Axis-Aligned Bounding Box (AABB)
    // Computes the smallest rectangle with sides parallel to the coordinate axes
    // that encloses all given points.
    std::cout << "SECTION 10: Axis-Aligned Bounding Box (AABB)" << std::endl;
    std::cout << "-------------------------------------------" << std::endl;

    if (points.empty()) {
        std::cout << "  No points to compute bounding box for." << std::endl;
    } else {
        // CGAL provides a convenient way to get the bounding box of a range of points.
        K::Iso_rectangle_2 bbox = CGAL::bbox_2(points.begin(), points.end());

        std::cout << "  Axis-Aligned Bounding Box:" << std::endl;
        std::cout << "    Min X: " << bbox.xmin() << std::endl;
        std::cout << "    Min Y: " << bbox.ymin() << std::endl;
        std::cout << "    Max X: " << bbox.xmax() << std::endl;
        std::cout << "    Max Y: " << bbox.ymax() << std::endl;
        std::cout << "    Bottom-left corner: "; print_point(bbox.min()); std::cout << std::endl;
        std::cout << "    Top-right corner: "; print_point(bbox.max()); std::cout << std::endl;
        std::cout << "    Width: " << bbox.xmax() - bbox.xmin() << std::endl;
        std::cout << "    Height: " << bbox.ymax() - bbox.ymin() << std::endl;
    }
    std::cout << std::endl;

    std::cout << "--- End of Computational Geometry Toolkit Demonstration ---" << std::endl;

    return 0;
}
