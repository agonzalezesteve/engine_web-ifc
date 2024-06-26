/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#pragma once

#include <fuzzy/geometry.h>

namespace webifc::geometry
{

	struct GeometryWithId
	{
		size_t id;
		fuzzybools::Geometry geometry;
	};

	struct BuildingElement : GeometryWithId
	{
		float thickness;
		std::vector<size_t> firstLevelBoundaries;
		bool isVoid;
		std::vector<size_t> voids;
	};

	struct SpaceOrBuilding : GeometryWithId
	{
		bool isSpace;
	};

	struct FirstLevelBoundary : GeometryWithId
	{
		glm::vec3 point;
		glm::vec3 normal;
		size_t buildingElement;
		size_t space;
	};

	enum class IfcInternalOrExternalEnum
	{
		INTERNAL,
		EXTERNAL,
		NOTDEFINED
	};

	struct SecondLevelBoundary : FirstLevelBoundary
	{
		IfcInternalOrExternalEnum boundaryCondition;
		int parentBoundary;

		std::string IfcInternalOrExternalEnum()
		{
			switch (boundaryCondition)
			{
			case IfcInternalOrExternalEnum::INTERNAL:
				return "INTERNAL";
			case IfcInternalOrExternalEnum::EXTERNAL:
				return "EXTERNAL";
			case IfcInternalOrExternalEnum::NOTDEFINED:
				return "NOTDEFINED";
			default:
				return "UNKNOWN";
			}
		}
	};

	class IfcGeometrySpace
	{
	public:
		void GetSpacesGeomsByBuildingElements(std::vector<BuildingElement> &buildingElements);

	private:
		std::vector<SpaceOrBuilding> GetSpacesAndBuildings(const Geometry &unionGeom);
		std::pair<Geometry, Geometry> SplitFirstBoundaryInIntersectionAndDifference(const Geometry &A, const Geometry &B);
		std::vector<Geometry> SplitGeometryByContiguousAndCoplanarFaces(const Geometry &geom);
		std::vector<FirstLevelBoundary> GetFirstLevelBoundaries(std::vector<BuildingElement> &buildingElements, const std::vector<SpaceOrBuilding> &spaceAndBuildings);
		void CorrectInternalSecondLevelBoundaries(std::vector<SecondLevelBoundary> &secondLevelBoundaries, const size_t buildingElementId, const std::vector<BuildingElement> &buildingElements);
		Geometry IntersectFirstBoundaryWithSecondGeometry(const Geometry &A, const Geometry &B),
			void AddVoids(const std::vector<BuildingElement> &buildingElements, const size_t buildingElementId, std::vector<SecondLevelBoundary> &secondLevelBoundaries);
		std::vector<SecondLevelBoundary> GetSecondLevelBoundaries(std::vector<BuildingElement> &buildingElements, const std::vector<SpaceOrBuilding> &spaceAndBuildings, std::vector<FirstLevelBoundary> &firstLevelBoundaries);
		bool ArePointsCollinear(const std::vector<fuzzybools::Point> &points, size_t idA, size_t idB, size_t idC);
		void TryAddPoint(size_t pointId, std::vector<size_t> &wire, const std::vector<fuzzybools::Point> &points, bool &isWireClosed);
		double WireArea(const std::vector<size_t> &wire, const std::vector<glm::dvec3> &points3D);
		double PolygonArea(std::pair<std::vector<glm::dvec3>, std::vector<std::vector<size_t>>> polygon);
		glm::vec3 GetWireNormal(const std::vector<size_t> &wire, const std::vector<fuzzybools::Point> &points);
		std::vector<std::pair<std::vector<glm::dvec3>, std::vector<std::vector<size_t>>>> SplitGeometryInPolygons(const Geometry &A);
	}

}