#pragma once

#include "CoreMinimal.h"
#include "Dynamicmesh/DynamicMesh3.h"
#include "functional"
#include "numeric"


struct FVoxelGrid;

/**
 * Stores the dimensions of the voxel grid and each voxel
 */
struct FVoxelGridInfo
{
	FIntVector GridSize; // Represents the number of voxels in the grid along each axis
	uint16 VoxelSize = 1; // The "Level" of the Voxel. 1,2,3... etc. Will always be a factor of GridSize

	explicit FVoxelGridInfo(const FIntVector& NewGridSize, const uint16 NewVoxelSize = 1)
		: GridSize(NewGridSize), VoxelSize(NewVoxelSize)
	{
		
	}
	

	int32 get_step() const
	{
		if (VoxelSize <= 1) return 1;

		int32 step = 1;
		int32 factor_count = 0;

		for (int32 i = 1; i <= FMath::Min3(GridSize.X, GridSize.Y, GridSize.Z) && factor_count < VoxelSize; ++i)
		{
			if (GridSize.X % i == 0 && GridSize.Y % i == 0 && GridSize.Z % i == 0)
			{
				++factor_count;
				step = i;
			}
		}

		return step;
	}
};

/**
 * Defines a voxel grid structure with functionality to read and write voxel data.
 */
struct FVoxelGrid
{
	FIntVector Resolution; // Represents the dimensions of the voxel grid.
	TArray<float> Data;    // Stores the voxel data in a flat array. Ensures that the array is stored in a contiguous block of memory, is cache-friendly, and can be easily passed to the surface nets algorithm.
	FVoxelGrid() : FVoxelGrid(FIntVector(1, 1, 1)) {}

	// Constructor to initialize the voxel grid with a specific resolution.
	explicit FVoxelGrid(const FIntVector& NewResolution)
		: Resolution(NewResolution + FIntVector(1, 1, 1))
	{
		Data.Init(0.f, Resolution.X * Resolution.Y * Resolution.Z);
	}
	
	// Reads the voxel value at a given (X, Y, Z) position. 
	float Read(const float X, const float Y, const float Z) const
	{
		return Data[GetIndex(X, Y, Z)];
	}
	float Read(const FVector& Position) const
	{
		return Read(Position.X, Position.Y, Position.Z);
	}

	// Writes a voxel value at a given (X, Y, Z) position. 
	void Write(const float X, const float Y, const float Z, const float Value)
	{
		Data[GetIndex(X, Y, Z)] = Value;
	}
	void Write(const FVector& Position, const float Value)
	{
		Write(Position.X, Position.Y, Position.Z, Value);
	}
	
private:
	// Calculates the flat array index for a given (X, Y, Z) position.
	int32 GetIndex(const float X, const float Y, const float Z) const
	{
		return X + (Y * Resolution.X) + (Z * Resolution.X * Resolution.Y);
	}
	
};

namespace SurfaceNets
{
	/**
	 * Implementation of the naive surface nets algorithm. Handles the generation of the mesh for the given voxel grid.
	 * @param	grid_info		 Information about the voxel grid and each voxel.
	 * @param	implicit_function The implicit function to use for the surface nets algorithm. Used to fix seams in the mesh.
	 * @param	dynamic_mesh	 Reference to FDynamicMesh3 where we will store the generated mesh.
	 * @param	surface_value	 The level-set of an implicit function for which the surface net algorithm extracts an isosurface. Ex : The surface net algorithm will extract the surface where the implicit function equals 0.
	 * @param   Origin			 The origin of the voxel grid in 3D space. Usually the location of the actor + the offset of the grid. Used for offsetting the vertex positions.
	 * @param	NoiseOrigin		 The origin of the noise in 3D space. Used for offsetting the noise in the implicit function. 
	 */
	void surface_nets(
		const FVoxelGridInfo& grid_info,
		const std::function<float(float, float, float)>& implicit_function,
		UE::Geometry::FDynamicMesh3& dynamic_mesh,
		const float surface_value = 0.f,
		const FVector Origin = FVector::ZeroVector,
		const FVector NoiseOrigin = FVector::ZeroVector
	);

	//-------------------------------------HELPER FUNCTIONS-------------------------------------//

	
	/**
	 * 
	 * @param x			The x coordinate of the voxel
	 * @param y			The y coordinate of the voxel
	 * @param z			The z coordinate of the voxel
	 * @param grid_info The information about the voxel grid
	 * @return			Index of the voxel in the flat array
	 */
	inline std::size_t GetIndex(const std::size_t x, const std::size_t y, const std::size_t z, const FVoxelGridInfo& grid_info)
	{
		return
			x +
			(y * (grid_info.GridSize.X + 1)) +
			(z * (grid_info.GridSize.X + 1) * (grid_info.GridSize.Y + 1));
	}
	
	
	
	/**
	 * 
	 * @param active_cube_index The Index of the active cube in the flat array
	 * @param grid_info			The information about the voxel grid
	 * @return					Tuple containing the x, y and z coordinates of the active cube
	 */
	inline std::tuple<std::size_t, std::size_t, std::size_t> GetCoordinates(std::size_t active_cube_index, const FVoxelGridInfo& grid_info)
	{
		std::size_t x = active_cube_index %  (grid_info.GridSize.X + 1);
		std::size_t y = active_cube_index /  (grid_info.GridSize.X + 1) % (grid_info.GridSize.Y + 1);
		std::size_t z = active_cube_index / ((grid_info.GridSize.X + 1) * (grid_info.GridSize.Y + 1));
		return std::make_tuple(x, y, z);
	}

	
	/**
	 * 
	 * @param scalar1		The scalar value of the first vertex
	 * @param scalar2		The scalar value of the second vertex
	 * @param surface_level The level-set of an implicit function for which the surface net algorithm extracts an isosurface.
	 * @return				Boolean indicating if the edge is bipolar
	 */
	inline bool are_edge_scalars_bipolar(float scalar1, float scalar2, float surface_level)
	{
		return scalar1 >= surface_level != scalar2 >= surface_level;
	}

	

	// Edge connections of the cube
	constexpr std::size_t edges[12][2] =
	{
		{ 0u, 1u },
		{ 1u, 2u },
		{ 2u, 3u },
		{ 3u, 0u },
		{ 4u, 5u },
		{ 5u, 6u },
		{ 6u, 7u },
		{ 7u, 4u },
		{ 0u, 4u },
		{ 1u, 5u },
		{ 2u, 6u },
		{ 3u, 7u }
	};

	
}




