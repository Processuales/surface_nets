
#include "NaiveSurfaceNets/surface_nets.h"




void SurfaceNets::surface_nets(
	const FVoxelGridInfo& grid_info,
	const std::function<float(float, float, float)>& implicit_function,
	UE::Geometry::FDynamicMesh3& dynamic_mesh,
	const float surface_value,
	const FVector Origin,
	const FVector NoiseOrigin)
{
	dynamic_mesh.Clear();
	dynamic_mesh.EnableAttributes();
	dynamic_mesh.EnableVertexNormals(FVector3f::Zero());
	
	std::unordered_map<int32, uint64> active_cube_to_vertex_index_map; 
	FVoxelGrid VoxelGrid(grid_info.GridSize); 
	const int32 step = grid_info.get_step();

	// step is 1, NoiseOrigin is 0,0,0    <---- Testing Purposes
	
	// Step 1: Fill the voxel grid with data, starting from Origin. -----------------------------------------------------------------
	for (std::size_t z = 0; z <= grid_info.GridSize.Z; z += step)
	for (std::size_t y = 0; y <= grid_info.GridSize.Y; y += step)	
	for (std::size_t x = 0; x <= grid_info.GridSize.X; x += step)
	{
		// NoiseOrigin will act like an offset to the implicit function
		const FVector position = FVector(x, y, z) + NoiseOrigin;
		VoxelGrid.Write(x,y,z, implicit_function(position.X, position.Y, position.Z));
	}
	
	
	// Step 2: Vertex generation and placement                      -----------------------------------------------------------------
	for (std::size_t z = 0; z < grid_info.GridSize.Z; z += step)
	for (std::size_t y = 0; y < grid_info.GridSize.Y; y += step)
	for (std::size_t x = 0; x < grid_info.GridSize.X; x += step)
	{
		// Corner positions of the cube in grid coordinates
		const FVector corners[8] = 
		{
			{ static_cast<float>(x),        static_cast<float>(y),        static_cast<float>(z)        }, 
			{ static_cast<float>(x + step), static_cast<float>(y),        static_cast<float>(z)        }, 
			{ static_cast<float>(x + step), static_cast<float>(y + step), static_cast<float>(z)        }, 
			{ static_cast<float>(x),        static_cast<float>(y + step), static_cast<float>(z)        },
			{ static_cast<float>(x),        static_cast<float>(y),        static_cast<float>(z + step) },
			{ static_cast<float>(x + step), static_cast<float>(y),        static_cast<float>(z + step) },
			{ static_cast<float>(x + step), static_cast<float>(y + step), static_cast<float>(z + step) },
			{ static_cast<float>(x),        static_cast<float>(y + step), static_cast<float>(z + step) },
		};
		
		// Evaluate the implicit function at each voxel corner
		const float voxel_corner_values[8] =
		{
			VoxelGrid.Read(corners[0]),
			VoxelGrid.Read(corners[1]),
			VoxelGrid.Read(corners[2]),
			VoxelGrid.Read(corners[3]),
			VoxelGrid.Read(corners[4]),
			VoxelGrid.Read(corners[5]),
			VoxelGrid.Read(corners[6]),
			VoxelGrid.Read(corners[7])
		};
		
		
		// Check for bipolar edges. Bipolar edges are edges that have one end inside the surface and the other outside. So a sign change in the scalar values.
		const bool edge_bipolarity_array[12] =
		{
			are_edge_scalars_bipolar(voxel_corner_values[edges[0][0]], voxel_corner_values[edges[0][1]], surface_value),
			are_edge_scalars_bipolar(voxel_corner_values[edges[1][0]], voxel_corner_values[edges[1][1]], surface_value),
			are_edge_scalars_bipolar(voxel_corner_values[edges[2][0]], voxel_corner_values[edges[2][1]], surface_value),
			are_edge_scalars_bipolar(voxel_corner_values[edges[3][0]], voxel_corner_values[edges[3][1]], surface_value),
			are_edge_scalars_bipolar(voxel_corner_values[edges[4][0]], voxel_corner_values[edges[4][1]], surface_value),
			are_edge_scalars_bipolar(voxel_corner_values[edges[5][0]], voxel_corner_values[edges[5][1]], surface_value),
			are_edge_scalars_bipolar(voxel_corner_values[edges[6][0]], voxel_corner_values[edges[6][1]], surface_value),
			are_edge_scalars_bipolar(voxel_corner_values[edges[7][0]], voxel_corner_values[edges[7][1]], surface_value),
			are_edge_scalars_bipolar(voxel_corner_values[edges[8][0]], voxel_corner_values[edges[8][1]], surface_value),
			are_edge_scalars_bipolar(voxel_corner_values[edges[9][0]], voxel_corner_values[edges[9][1]], surface_value),
			are_edge_scalars_bipolar(voxel_corner_values[edges[10][0]],voxel_corner_values[edges[10][1]],surface_value),
			are_edge_scalars_bipolar(voxel_corner_values[edges[11][0]],voxel_corner_values[edges[11][1]],surface_value),
		};

		// Mark the voxel as active if it contains any bipolar edges
		bool const is_voxel_active = edge_bipolarity_array[0] ||
			edge_bipolarity_array[1]  ||
			edge_bipolarity_array[2]  ||
			edge_bipolarity_array[3]  ||
			edge_bipolarity_array[4]  ||
			edge_bipolarity_array[5]  ||
			edge_bipolarity_array[6]  ||
			edge_bipolarity_array[7]  ||
			edge_bipolarity_array[8]  ||
			edge_bipolarity_array[9]  ||
			edge_bipolarity_array[10] ||
			edge_bipolarity_array[11];

		// If the voxel is not active, skip to the next voxel
		if (!is_voxel_active) 
			continue;
		

		std::vector<FVector> edge_intersection_points;

		// We visit every bipolar edge
		for (std::size_t edge = 0; edge < 12; ++edge)
		{
			if (!edge_bipolarity_array[edge]) 
				continue; // This is not a bipolar edge, skip to the next edge
 
			// Get points p1, p2 of the edge in grid coordinates
			const FVector p1 = corners[edges[edge][0]];
			const FVector p2 = corners[edges[edge][1]];

			// Get value of the implicit function for the corresponding vertices
			const float s1 = voxel_corner_values[edges[edge][0]];
			const float s2 = voxel_corner_values[edges[edge][1]];

			// Perform linear interpolation 
			const float t = (surface_value - s1) / (s2 - s1);
			edge_intersection_points.emplace_back(
				p1 + t * (p2 - p1)
			);
		}

		// Calculate the geometric center of intersection points
		const float number_of_intersection_points = static_cast<float>(edge_intersection_points.size());
		const FVector sum_of_intersection_points = std::accumulate(
			edge_intersection_points.cbegin(),
			edge_intersection_points.cend(),
			FVector::ZeroVector);

		const FVector geometric_center_of_edge_intersection_points = sum_of_intersection_points / number_of_intersection_points;

		// Construct the vertex position in global coordinates. Multiply by 100 to scale the mesh and add the origin
		const FVector3d mesh_vertex =
		{
			geometric_center_of_edge_intersection_points.X * 100 + Origin.X,
			geometric_center_of_edge_intersection_points.Y * 100 + Origin.Y,
			geometric_center_of_edge_intersection_points.Z * 100 + Origin.Z,
		};

		
		// Map the active voxel to the new vertex index
		// Record the correspondence between the voxel's index in the grid and the vertex's index in the mesh for later use in triangulation
		const int32 active_cube_index = GetIndex(x, y, z, grid_info);
		const int64 vertex_index = dynamic_mesh.AppendVertex(mesh_vertex);
		active_cube_to_vertex_index_map[active_cube_index] = vertex_index;

		// Calculate the normal
		FVector gradient = FVector::ZeroVector;
		
		// Approximate the gradient in x direction
		gradient.X = (voxel_corner_values[1] - voxel_corner_values[0]) + 
			         (voxel_corner_values[2] - voxel_corner_values[3]) + 
			         (voxel_corner_values[5] - voxel_corner_values[4]) + 
			         (voxel_corner_values[6] - voxel_corner_values[7]);

		// Approximate the gradient in y direction
		gradient.Y = (voxel_corner_values[3] - voxel_corner_values[0]) + 
					 (voxel_corner_values[2] - voxel_corner_values[1]) + 
					 (voxel_corner_values[7] - voxel_corner_values[4]) + 
					 (voxel_corner_values[6] - voxel_corner_values[5]);

		// Approximate the gradient in z direction
		gradient.Z = (voxel_corner_values[4] - voxel_corner_values[0]) + 
					 (voxel_corner_values[5] - voxel_corner_values[1]) + 
					 (voxel_corner_values[6] - voxel_corner_values[2]) + 
					 (voxel_corner_values[7] - voxel_corner_values[3]);

		FVector normal = gradient.GetSafeNormal();
		dynamic_mesh.SetVertexNormal(vertex_index, FVector3f(normal.X, normal.Y, normal.Z));
	}
	

	// Step 3: Triangulation of the active cubes                    -----------------------------------------------------------------
	for (const auto& key_value : active_cube_to_vertex_index_map)
	{
		// Get the index and the vertex index of the active cube
		const int32 active_cube_index = key_value.first;
		const uint64 vertex_index = key_value.second;

		// Get the coordinates in 3D
		const std::tuple<size_t, size_t, size_t> xyz = GetCoordinates(active_cube_index, grid_info);
		std::size_t x = std::get<0>(xyz);
		std::size_t y = std::get<1>(xyz);
		std::size_t z = std::get<2>(xyz);

		// Skip the bottom voxels since they are not triangulated. Not too sure if we need this, but removing it will make no difference.
		//if (x == 0 || y == 0 || z == 0) 
			//continue;

		/*    From this Active cube
		*
		*        7          6
		*        o----------o
		*       /|         /|
		*     4/ |       5/ |
		*     o--|-------o  |
		*     |  o-------|--o
		*     | /3       | /2
		*     |/         |/
		*     o----------o
		*     0          1
		*
		* The corners we need are 0, 4, 3, 1
		*
		*  4
		*  o
		*  |  o
		*  | /3
		*  |/
		*  o----------o
		*  0          1
		*
		*  
		*/


		const FVector voxel_corners_of_interest[4] = {
			FVector(x       , y       , z       ), // Corner 0
			FVector(x       , y       , z + step), // Corner 4
			FVector(x       , y + step, z       ), // Corner 3
			FVector(x + step, y       , z       )  // Corner 1
		};

		// Read the scalar values of both points of 3 edges. 
		const float edge_scalar_values[3][2] = {
			{ // Vertices 0 and 4
				VoxelGrid.Read(voxel_corners_of_interest[0]),
				VoxelGrid.Read(voxel_corners_of_interest[1])
			},
			{ // Vertices 3 and 0
				VoxelGrid.Read(voxel_corners_of_interest[2]),
				VoxelGrid.Read(voxel_corners_of_interest[0])
			},
			{ // Vertices 0 and 1
				VoxelGrid.Read(voxel_corners_of_interest[0]),
				VoxelGrid.Read(voxel_corners_of_interest[3])
			}
		};


		/*
		*
		*           o----------o
		*          /|         /|         
		*         / |   0    / |  (x,y,z)        <---- This is the current active cube (x,y,z) in the above coordinate frame.
		*        o--|-------o--|-------o               Top ones are 0,1,2
		*       /|  o------/|--o------/|--o            Bottom ones are 3,4,5
		*      / | /|1    / | /|2    / | /|
		*     o--|/-|--5-o--|/-|--4-o  |/ |
		*     |  o--|----|--o--|----|--o  |
		*     | /|  o----|-/|--o----|-/|--o
		*     |/ | /     |/ | /3    |/ | /
		*     o--|/------o--|/------o  |/
		*        o-------|--o-------|--o
		*                | /        | /
		*                |/         |/
		*                o----------o
		*
		* 
		*/

		// Here are the coordinates of the 6 neighbours of the active cube (x,y,z)
		const std::size_t neighbor_grid_positions[6][3] =
		{
			{x - step, y       , z       }, // Neighbour 0
			{x - step, y - step, z       }, // Neighbour 1
			{x       , y - step, z       }, // Neighbour 2
			{x       , y - step, z - step}, // Neighbour 3
			{x       , y       , z - step}, // Neighbour 4
			{x - step, y       , z - step}  // Neighbour 5
		};

		

		
		// Possible neighbours of the active cube that form a quad
		const std::size_t quad_neighbors[3][3] = {
			{0, 1, 2}, // Neighbour 0, 1 and 2 form a quad
			{0, 5, 4}, // Neighbour 0, 5 and 4 form a quad
			{2, 3, 4}  // Neighbour 2, 3 and 4 form a quad
		};


		// Possible winding orders of the neighbours. Anti-clockwise and clockwise
		const TArray<std::size_t> quad_neighbor_orders[2] =
		{
			{ 2, 1, 0 }, 
			{ 0, 1, 2 }  
		};

		// For each quad, we will triangulate
		for (std::size_t quad = 0; quad < 3; quad++)
		{
			// Get the indices of the neighbours. 3 neighbours per loop, 3 loops. A total of 9 checks.
			/* Here are the loop iterations : 		    ||                              ||
			 * First iteration  : x - step, y       , z || x - step, y - step, z        || x, y - step, z
			 * Second iteration : x - step, y       , z || x - step, y       , z - step || x, y       , z - step
			 * Third iteration  : x       , y - step, z || x       , y - step, z - step || x, y       , z - step
			 *                                          ||                              ||
			 */
			const size_t neighbor1 = GetIndex(
				neighbor_grid_positions[quad_neighbors[quad][0]][0],
				neighbor_grid_positions[quad_neighbors[quad][0]][1],
				neighbor_grid_positions[quad_neighbors[quad][0]][2],
				grid_info);

			const size_t neighbor2 = GetIndex(
				neighbor_grid_positions[quad_neighbors[quad][1]][0],
				neighbor_grid_positions[quad_neighbors[quad][1]][1],
				neighbor_grid_positions[quad_neighbors[quad][1]][2],
				grid_info);

			const size_t neighbor3 = GetIndex(
				neighbor_grid_positions[quad_neighbors[quad][2]][0],
				neighbor_grid_positions[quad_neighbors[quad][2]][1],
				neighbor_grid_positions[quad_neighbors[quad][2]][2],
				grid_info);

			
			// Only triangulate if all the neighbours are active (they have a vertex)
			if (!active_cube_to_vertex_index_map.contains(neighbor1) ||
				!active_cube_to_vertex_index_map.contains(neighbor2) ||
				!active_cube_to_vertex_index_map.contains(neighbor3))
				continue;

			// Get the vertex indices of the neighbours
			const std::size_t neighbor_vertices[3] = {
				active_cube_to_vertex_index_map.at(neighbor1),
				active_cube_to_vertex_index_map.at(neighbor2),
				active_cube_to_vertex_index_map.at(neighbor3)
			};

			// Get the correct winding order
			auto const& neighbor_vertices_order =
				edge_scalar_values[quad][1] > edge_scalar_values[quad][0] ?
				quad_neighbor_orders[0] :
				quad_neighbor_orders[1];

			// Get the vertex index of the current active cube
			uint64 v0 = vertex_index;
			const size_t v1 = neighbor_vertices[neighbor_vertices_order[0]];
			const size_t v2 = neighbor_vertices[neighbor_vertices_order[1]];
			const size_t v3 = neighbor_vertices[neighbor_vertices_order[2]];

			dynamic_mesh.AppendTriangle(v0, v1, v2);
			dynamic_mesh.AppendTriangle(v0, v2, v3);
		}
	}
}





