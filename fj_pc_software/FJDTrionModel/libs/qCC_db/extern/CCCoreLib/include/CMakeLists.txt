# SPDX-License-Identifier: MIT
# Copyright © Andy Maloney <asmaloney@gmail.com>

file(GLOB headerFiles CMAKE_CONFIGURE_DEPENDS *.h )
target_sources( ${PROJECT_NAME}
	PRIVATE ${headerFiles})

install(
	FILES
		AutoSegmentationTools.h
		BoundingBox.h
		CCConst.h
		CCCoreLib.h
		CCGeom.h
		CCMath.h
		CCMiscTools.h
		CCPlatform.h
		CCShareable.h
		CCToolbox.h
		CCTypes.h
		ChamferDistanceTransform.h
		CloudSamplingTools.h
		ConjugateGradient.h
		Delaunay2dMesh.h
		DgmOctree.h
		DgmOctreeReferenceCloud.h
		DistanceComputationTools.h
		ErrorFunction.h
		FastMarching.h
		FastMarchingForPropagation.h
		Garbage.h
		GenericCloud.h
		GenericDistribution.h
		GenericIndexedCloud.h
		GenericIndexedCloudPersist.h
		GenericIndexedMesh.h
		GenericMesh.h
		GenericOctree.h
		GenericProgressCallback.h
		GenericTriangle.h
		GeometricalAnalysisTools.h
		GridAndMeshIntersection.h
		Grid3D.h
		Jacobi.h
		KdTree.h
		LocalModel.h
		ManualSegmentationTools.h
		MathTools.h
		MeshSamplingTools.h
		Neighbourhood.h
		NormalDistribution.h
		ParallelSort.h
		PointCloud.h
		PointCloudTpl.h
		PointProjectionTools.h
		Polyline.h
		RayAndBox.h
		ReferenceCloud.h
		RegistrationTools.h
		SaitoSquaredDistanceTransform.h
		ScalarField.h
		ScalarFieldTools.h
		SimpleMesh.h
		SimpleTriangle.h
		SquareMatrix.h
		StatisticalTestingTools.h
		TrueKdTree.h
		WeibullDistribution.h
	DESTINATION
		include/CCCoreLib
)

target_include_directories( ${PROJECT_NAME}
	PUBLIC
		$<INSTALL_INTERFACE:include/CCCoreLib>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

