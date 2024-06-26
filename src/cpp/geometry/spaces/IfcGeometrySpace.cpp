/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#include "IfcGeometrySpace.h"

namespace webifc::geometry
{

    std::vector<SpaceOrBuilding> IfcGeometrySpace::GetSpacesAndBuildings(const fuzzybools::Geometry &unionGeom)
    {
        std::vector<SpaceOrBuilding> spacesAndBuildings;

        fuzzybools::SharedPosition sp;
        sp.AddGeometryA(unionGeom);

        std::vector<Triangle> geomTriangles = sp.A.triangles;
        std::vector<bool> visited(geomTriangles.size(), false);
        for (size_t triangleId = 0; triangleId < geomTriangles.size(); triangleId++)
        {
            if (visited[triangleId])
                continue;

            fuzzybools::Geometry spaceOrBuildingGeom;
            std::queue<size_t> q;
            q.push(triangleId);

            while (!q.empty())
            {
                size_t currentId = q.front();
                q.pop();
                if (visited[currentId])
                    continue;

                visited[currentId] = true;

                Triangle triangle = geomTriangles[currentId];
                glm::dvec3 a = sp.points[triangle.a].location3D;
                glm::dvec3 b = sp.points[triangle.b].location3D;
                glm::dvec3 c = sp.points[triangle.c].location3D;
                spaceOrBuildingGeom.AddFace(a, b, c);

                for (const auto &neigthbourTriangle : sp.A.GetNeighbourTriangles(triangle))
                {
                    for (size_t i = 0; i < neigthbourTriangle.second.size(); i++)
                    {
                        size_t neighbourId = neigthbourTriangle.second[i];
                        if (visited[neighbourId])
                            continue;

                        q.push(neighbourId);
                    }
                }
            }

            SpaceOrBuilding spaceOrBuilding;
            spaceOrBuilding.id = spacesAndBuildings.size();
            spaceOrBuilding.geometry = spaceOrBuildingGeom;
            spaceOrBuilding.isSpace = (spaceOrBuildingGeom.Volume() < 0);

            spacesAndBuildings.push_back(spaceOrBuilding);
        }

        return spacesAndBuildings;
    }

    std::pair<fuzzybools::Geometry, fuzzybools::Geometry> IfcGeometrySpace::SplitFirstBoundaryInIntersectionAndDifference(const fuzzybools::Geometry &A, const fuzzybools::Geometry &B)
    {
        fuzzybools::SharedPosition sp;
        sp.Construct(A, B);

        auto bvh1 = fuzzybools::MakeBVH(A);
        auto bvh2 = fuzzybools::MakeBVH(B);

        auto geom = Normalize(sp);

        auto firstBoundarySplitted = fuzzybools::SplitFirstBoundary(geom, bvh1, bvh2);
        return {firstBoundarySplitted.first, firstBoundarySplitted.second.first};
    }

    std::vector<fuzzybools::Geometry> IfcGeometrySpace::SplitGeometryByContiguousAndCoplanarFaces(const fuzzybools::Geometry &geom)
    {
        std::vector<fuzzybools::Geometry> newGeoms;
        if (geom.IsEmpty())
            return newGeoms;

        fuzzybools::SharedPosition sp;
        sp.AddGeometryA(geom);

        std::vector<Triangle> geomTriangles = sp.A.triangles;
        std::vector<bool> visited(geomTriangles.size(), false);
        for (size_t triangleId = 0; triangleId < geomTriangles.size(); triangleId++)
        {
            if (visited[triangleId])
                continue;

            fuzzybools::Geometry newGeom;
            std::queue<size_t> q;
            q.push(triangleId);

            while (!q.empty())
            {
                size_t currentId = q.front();
                q.pop();
                if (visited[currentId])
                    continue;

                visited[currentId] = true;

                Triangle triangle = geomTriangles[currentId];
                glm::dvec3 norm = sp.GetNormal(triangle);

                glm::dvec3 a = sp.points[triangle.a].location3D;
                glm::dvec3 b = sp.points[triangle.b].location3D;
                glm::dvec3 c = sp.points[triangle.c].location3D;
                newGeom.AddFace(a, b, c);

                for (auto &neigthbourTriangle : sp.A.GetNeighbourTriangles(triangle))
                {
                    for (size_t i = 0; i < neigthbourTriangle.second.size(); i++)
                    {
                        size_t neighbourId = neigthbourTriangle.second[i];

                        if (std::fabs(glm::dot(norm, sp.GetNormal(geomTriangles[neighbourId])) - 1) > EPS_BIG)
                            continue;

                        if (visited[neighbourId])
                            continue;

                        q.push(neighbourId);
                    }
                }
            }

            newGeoms.push_back(newGeom);
        }

        return newGeoms;
    }

    std::vector<FirstLevelBoundary> IfcGeometrySpace::GetFirstLevelBoundaries(std::vector<BuildingElement> &buildingElements, const std::vector<SpaceOrBuilding> &spaceAndBuildings)
    {
        std::vector<FirstLevelBoundary> firstLevelBoundaries;

        for (size_t spaceId = 0; spaceId < spaceAndBuildings.size(); spaceId++)
        {
            auto geom = spaceAndBuildings[spaceId].geometry;

            for (size_t buildingElementId = 0; buildingElementId < buildingElements.size(); buildingElementId++)
            {
                auto &buildingElement = buildingElements[buildingElementId];
                if (buildingElement.isVoid)
                    continue;

                auto intersectionAndDifferenceGeoms = SplitFirstBoundaryInIntersectionAndDifference(geom, buildingElement.geometry);
                for (auto firstLevelBoundaryGeom : SplitGeometryByContiguousAndCoplanarFaces(intersectionAndDifferenceGeoms.first))
                {
                    Face tri = firstLevelBoundaryGeom.GetFace(0);
                    auto a = firstLevelBoundaryGeom.GetPoint(tri.i0);
                    auto b = firstLevelBoundaryGeom.GetPoint(tri.i1);
                    auto c = firstLevelBoundaryGeom.GetPoint(tri.i2);
                    glm::dvec3 norm;
                    computeSafeNormal(a, b, c, norm, EPS_SMALL);

                    FirstLevelBoundary firstLevelBoundary;
                    firstLevelBoundary.id = firstLevelBoundaries.size();
                    firstLevelBoundary.geometry = firstLevelBoundaryGeom;
                    firstLevelBoundary.point = a;
                    firstLevelBoundary.normal = norm;
                    firstLevelBoundary.buildingElement = buildingElementId;
                    firstLevelBoundary.space = spaceId;

                    firstLevelBoundaries.push_back(firstLevelBoundary);
                    buildingElement.firstLevelBoundaries.push_back(firstLevelBoundary.id);
                }

                geom = intersectionAndDifferenceGeoms.second;
                if (geom.IsEmpty())
                    break;
            }
        }

        return firstLevelBoundaries;
    }

    void IfcGeometrySpace::CorrectInternalSecondLevelBoundaries(std::vector<SecondLevelBoundary> &secondLevelBoundaries, const size_t buildingElementId, const std::vector<BuildingElement> &buildingElements)
    {
        int secondLevelBoundaryId = secondLevelBoundaries.size() - 1;

        while (secondLevelBoundaryId >= 0)
        {
            auto &secondLevelBoundary = secondLevelBoundaries[secondLevelBoundaryId];
            if (secondLevelBoundary.buildingElement != buildingElementId)
                break;

            switch (secondLevelBoundary.boundaryCondition)
            {
            case IfcInternalOrExternalEnum::INTERNAL:
            {
                auto &otherSecondLevelBoundary = secondLevelBoundaries[secondLevelBoundaryId - 1];
                double internalSecondLevelBoundaryDistance = glm::dot(otherSecondLevelBoundary.normal, otherSecondLevelBoundary.point) - glm::dot(otherSecondLevelBoundary.normal, secondLevelBoundary.point);
                if (std::fabs(internalSecondLevelBoundaryDistance - buildingElements[buildingElementId].thickness) > EPS_SMALL)
                {
                    secondLevelBoundary.boundaryCondition = IfcInternalOrExternalEnum::NOTDEFINED;
                    otherSecondLevelBoundary.boundaryCondition = IfcInternalOrExternalEnum::NOTDEFINED;
                }

                secondLevelBoundaryId -= 2;
                break;
            }
            case IfcInternalOrExternalEnum::EXTERNAL:
            {
                secondLevelBoundaryId -= 2;
                break;
            }
            default:
                secondLevelBoundaryId -= 1;
                break;
            }
        }
    }

    fuzzybools::Geometry IfcGeometrySpace::IntersectFirstBoundaryWithSecondGeometry(const fuzzybools::Geometry &A, const fuzzybools::Geometry &B)
    {
        fuzzybools::SharedPosition sp;
        sp.Construct(A, B);

        auto bvh1 = fuzzybools::MakeBVH(A);
        auto bvh2 = fuzzybools::MakeBVH(B);

        auto geom = Normalize(sp);

        return fuzzybools::SplitFirstBoundary(geom, bvh1, bvh2).second.second;
    }

    void IfcGeometrySpace::AddVoids(const std::vector<BuildingElement> &buildingElements, const size_t buildingElementId, std::vector<SecondLevelBoundary> &secondLevelBoundaries)
    {
        auto buildingElement = buildingElements[buildingElementId];
        for (size_t i = 0; i < buildingElement.voids.size(); ++i)
        {
            auto voidElement = buildingElements[buildingElement.voids[i]];

            int secondLevelBoundaryId = secondLevelBoundaries.size() - 1;
            bool exitLoop = false;
            while (secondLevelBoundaryId >= 0 && !exitLoop)
            {
                auto parentBoundary = secondLevelBoundaries[secondLevelBoundaryId];
                switch (parentBoundary.boundaryCondition)
                {
                case IfcInternalOrExternalEnum::INTERNAL:
                case IfcInternalOrExternalEnum::EXTERNAL:
                {
                    auto secondLevelBoundaryGeom = IntersectFirstBoundaryWithSecondGeometry(parentBoundary.geometry, voidElement.geometry);
                    if (!secondLevelBoundaryGeom.IsEmpty())
                    {
                        SecondLevelBoundary secondLevelBoundary;
                        secondLevelBoundary.id = secondLevelBoundaries.size();
                        secondLevelBoundary.geometry = secondLevelBoundaryGeom;
                        secondLevelBoundary.point = secondLevelBoundary.geometry.GetPoint(secondLevelBoundary.geometry.GetFace(0).i0);
                        secondLevelBoundary.normal = parentBoundary.normal;
                        secondLevelBoundary.buildingElement = voidElement.id;
                        secondLevelBoundary.space = parentBoundary.space;
                        secondLevelBoundary.boundaryCondition = parentBoundary.boundaryCondition;
                        secondLevelBoundary.parentBoundary = parentBoundary.id;
                        secondLevelBoundaries.push_back(secondLevelBoundary);

                        auto otherParentBoundary = secondLevelBoundaries[secondLevelBoundaryId - 1];
                        double parentBoundaryDistance = glm::dot(parentBoundary.normal, parentBoundary.point) - glm::dot(parentBoundary.normal, otherParentBoundary.point);

                        SecondLevelBoundary otherSecondLevelBoundary;
                        otherSecondLevelBoundary.id = secondLevelBoundaries.size();
                        otherSecondLevelBoundary.geometry = secondLevelBoundary.geometry.Translate((float)parentBoundaryDistance * otherParentBoundary.normal);
                        otherSecondLevelBoundary.geometry.Flip();
                        otherSecondLevelBoundary.point = otherSecondLevelBoundary.geometry.GetPoint(otherSecondLevelBoundary.geometry.GetFace(0).i0);
                        otherSecondLevelBoundary.normal = otherParentBoundary.normal;
                        otherSecondLevelBoundary.buildingElement = voidElement.id;
                        otherSecondLevelBoundary.space = otherParentBoundary.space;
                        otherSecondLevelBoundary.boundaryCondition = otherParentBoundary.boundaryCondition;
                        otherSecondLevelBoundary.parentBoundary = otherParentBoundary.id;
                        secondLevelBoundaries.push_back(otherSecondLevelBoundary);

                        exitLoop = true;
                    }

                    secondLevelBoundaryId -= 2;
                    break;
                }
                case IfcInternalOrExternalEnum::NOTDEFINED:
                default:
                    secondLevelBoundaryId -= 1;
                    break;
                }
            }
        }
    }

    std::vector<SecondLevelBoundary> IfcGeometrySpace::GetSecondLevelBoundaries(std::vector<BuildingElement> &buildingElements, const std::vector<SpaceOrBuilding> &spaceAndBuildings, std::vector<FirstLevelBoundary> &firstLevelBoundaries)
    {
        std::vector<SecondLevelBoundary> secondLevelBoundaries;

        for (size_t buildingElementId = 0; buildingElementId < buildingElements.size(); buildingElementId++)
        {
            auto &buildingElement = buildingElements[buildingElementId];
            if (buildingElement.isVoid)
                continue;

            double maxArea = -1.0;
            for (size_t i = 0; i < buildingElement.firstLevelBoundaries.size(); i++)
            {
                auto &firstLevelBoundary = firstLevelBoundaries[buildingElement.firstLevelBoundaries[i]];
                if (firstLevelBoundary.geometry.IsEmpty())
                    continue;

                for (size_t j = i + 1; j < buildingElement.firstLevelBoundaries.size(); j++)
                {
                    auto &otherFirstLevelBoundary = firstLevelBoundaries[buildingElement.firstLevelBoundaries[j]];
                    if (glm::dot(firstLevelBoundary.normal, otherFirstLevelBoundary.normal) + 1 > EPS_SMALL)
                        continue;

                    double firstLevelBoundaryDistance = glm::dot(firstLevelBoundary.normal, firstLevelBoundary.point) - glm::dot(firstLevelBoundary.normal, otherFirstLevelBoundary.point);
                    if (firstLevelBoundaryDistance < EPS_SMALL)
                        continue;

                    auto intersectionAndDifferenceGeoms = SplitFirstBoundaryInIntersectionAndDifference(firstLevelBoundary.geometry, otherFirstLevelBoundary.geometry.Translate((float)firstLevelBoundaryDistance * firstLevelBoundary.normal));

                    for (auto secondLevelBoundaryGeom : SplitGeometryByContiguousAndCoplanarFaces(intersectionAndDifferenceGeoms.first))
                    {
                        IfcInternalOrExternalEnum boundaryCondition = ((!spaceAndBuildings[firstLevelBoundary.space].isSpace || !spaceAndBuildings[otherFirstLevelBoundary.space].isSpace) ? IfcInternalOrExternalEnum::EXTERNAL : IfcInternalOrExternalEnum::INTERNAL);

                        SecondLevelBoundary secondLevelBoundary;
                        secondLevelBoundary.id = secondLevelBoundaries.size();
                        secondLevelBoundary.geometry = secondLevelBoundaryGeom;
                        secondLevelBoundary.point = secondLevelBoundary.geometry.GetPoint(secondLevelBoundary.geometry.GetFace(0).i0);
                        secondLevelBoundary.normal = firstLevelBoundary.normal;
                        secondLevelBoundary.buildingElement = buildingElementId;
                        secondLevelBoundary.space = firstLevelBoundary.space;
                        secondLevelBoundary.boundaryCondition = boundaryCondition;
                        secondLevelBoundary.parentBoundary = -1;
                        secondLevelBoundaries.push_back(secondLevelBoundary);

                        SecondLevelBoundary otherSecondLevelBoundary;
                        otherSecondLevelBoundary.id = secondLevelBoundaries.size();
                        otherSecondLevelBoundary.geometry = secondLevelBoundary.geometry.Translate((float)firstLevelBoundaryDistance * otherFirstLevelBoundary.normal);
                        otherSecondLevelBoundary.geometry.Flip();
                        otherSecondLevelBoundary.point = otherSecondLevelBoundary.geometry.GetPoint(otherSecondLevelBoundary.geometry.GetFace(0).i0);
                        otherSecondLevelBoundary.normal = otherFirstLevelBoundary.normal;
                        otherSecondLevelBoundary.buildingElement = buildingElementId;
                        otherSecondLevelBoundary.space = otherFirstLevelBoundary.space;
                        otherSecondLevelBoundary.boundaryCondition = boundaryCondition;
                        otherSecondLevelBoundary.parentBoundary = -1;
                        secondLevelBoundaries.push_back(otherSecondLevelBoundary);

                        auto area = secondLevelBoundary.geometry.Area();
                        if (area > maxArea)
                        {
                            buildingElement.thickness = firstLevelBoundaryDistance;
                            maxArea = area;
                        }
                    }

                    firstLevelBoundary.geometry = intersectionAndDifferenceGeoms.second;
                    intersectionAndDifferenceGeoms = SplitFirstBoundaryInIntersectionAndDifference(otherFirstLevelBoundary.geometry, intersectionAndDifferenceGeoms.first.Translate((float)firstLevelBoundaryDistance * otherFirstLevelBoundary.normal));
                    otherFirstLevelBoundary.geometry = intersectionAndDifferenceGeoms.second;

                    if (firstLevelBoundary.geometry.IsEmpty())
                        break;
                }

                for (auto secondLevelBoundaryGeom : SplitGeometryByContiguousAndCoplanarFaces(firstLevelBoundary.geometry))
                {
                    SecondLevelBoundary secondLevelBoundary;
                    secondLevelBoundary.id = secondLevelBoundaries.size();
                    secondLevelBoundary.geometry = secondLevelBoundaryGeom;
                    secondLevelBoundary.point = secondLevelBoundary.geometry.GetPoint(secondLevelBoundary.geometry.GetFace(0).i0);
                    secondLevelBoundary.normal = firstLevelBoundary.normal;
                    secondLevelBoundary.buildingElement = buildingElementId;
                    secondLevelBoundary.space = firstLevelBoundary.space;
                    secondLevelBoundary.boundaryCondition = IfcInternalOrExternalEnum::NOTDEFINED;
                    secondLevelBoundary.parentBoundary = -1;
                    secondLevelBoundaries.push_back(secondLevelBoundary);
                }
            }

            CorrectInternalSecondLevelBoundaries(secondLevelBoundaries, buildingElementId, buildingElements);
            AddVoids(buildingElements, buildingElementId, secondLevelBoundaries);
        }

        return secondLevelBoundaries;
    }

    bool IfcGeometrySpace::ArePointsCollinear(const std::vector<fuzzybools::Point> &points, size_t idA, size_t idB, size_t idC)
    {
        auto A = points[idA].location3D;
        auto B = points[idB].location3D;
        auto C = points[idC].location3D;
        return glm::length(glm::cross(B - A, C - A)) < 1e-6;
    }

    void IfcGeometrySpace::TryAddPoint(size_t pointId, std::vector<size_t> &wire, const std::vector<fuzzybools::Point> &points, bool &isWireClosed)
    {
        if (pointId == wire[0])
        {
            if (ArePointsCollinear(points, wire[0], wire[wire.size() - 1], wire[wire.size() - 2]))
            {
                wire.pop_back();
            }
            if (ArePointsCollinear(points, wire[1], wire[0], wire[wire.size() - 1]))
            {
                wire.erase(wire.begin());
            }
            isWireClosed = true;
        }
        else
        {
            if (ArePointsCollinear(points, pointId, wire[wire.size() - 1], wire[wire.size() - 2]))
            {
                wire[wire.size() - 1] = pointId;
            }
            else
            {
                wire.push_back(pointId);
            }
        }
    }

    double IfcGeometrySpace::WireArea(const std::vector<size_t> &wire, const std::vector<glm::dvec3> &points3D)
    {
        if (wire.size() < 3)
            return 0.0;

        glm::dvec3 area = glm::cross(points3D[wire[0]], points3D[wire[wire.size() - 1]]);

        for (int i = 1; i < wire.size(); ++i)
        {
            area += glm::cross(points3D[wire[i]], points3D[wire[i - 1]]);
        }

        return glm::length(area) / 2;
    }

    double IfcGeometrySpace::PolygonArea(std::pair<std::vector<glm::dvec3>, std::vector<std::vector<size_t>>> polygon)
    {
        double area = WireArea(polygon.second[0], polygon.first);

        for (int i = 1; i < polygon.second.size(); ++i)
        {
            area -= WireArea(polygon.second[i], polygon.first);
        }

        return area;
    }

    glm::vec3 IfcGeometrySpace::GetWireNormal(const std::vector<size_t> &wire, const std::vector<fuzzybools::Point> &points)
    {
        if (wire.size() < 3)
            return glm::vec3(0, 0, 0);

        auto A = points[wire[0]].location3D;
        auto B = points[wire[1]].location3D;
        auto C = points[wire[-1]].location3D;

        return glm::cross(B - A, C - A);
    }

    std::vector<std::pair<std::vector<glm::dvec3>, std::vector<std::vector<size_t>>>> IfcGeometrySpace::SplitGeometryInPolygons(const fuzzybools::Geometry &A)
    {
        std::vector<std::pair<std::vector<glm::dvec3>, std::vector<std::vector<size_t>>>> polygons;

        for (auto contiguousAndCoplanarFaces : SplitGeometryByContiguousAndCoplanarFaces(A))
        {
            std::pair<std::vector<glm::dvec3>, std::vector<std::vector<size_t>>> polygon;

            fuzzybools::SharedPosition polygonSharedPosition;

            fuzzybools::SharedPosition sp;
            sp.AddGeometryA(contiguousAndCoplanarFaces);

            auto contoursA = sp.A.GetContourSegments();
            for (auto &[planeId, contours] : contoursA)
            {
                std::vector<bool> visited(contours.size(), false);
                for (int i = 0; i < contours.size(); ++i)
                {
                    if (visited[i])
                        continue;

                    std::vector<size_t> wire;

                    wire.push_back(contours[i].first);
                    wire.push_back(contours[i].second);
                    visited[i] = true;

                    bool isWireClosed = false;
                    while (!isWireClosed)
                    {
                        for (int j = i + 1; j < contours.size(); ++j)
                        {
                            if (visited[j])
                                continue;

                            if (contours[j].first == wire[wire.size() - 1])
                            {
                                TryAddPoint(contours[j].second, wire, sp.points, isWireClosed);
                                visited[j] = true;
                                break;
                            }
                            else if (contours[j].second == wire[wire.size() - 1])
                            {
                                TryAddPoint(contours[j].first, wire, sp.points, isWireClosed);
                                visited[j] = true;
                                break;
                            }
                        }
                    }

                    std::vector<size_t> polygonWire(wire.size());
                    for (int k = 0; k < wire.size(); ++k)
                    {
                        auto pointId = wire[k];
                        auto point3D = sp.points[pointId].location3D;
                        polygonWire[k] = polygonSharedPosition.AddPoint(point3D);
                    }

                    if (polygon.second.size() > 0 && glm::dot(GetWireNormal(polygon.second[0], sp.points), GetWireNormal(polygonWire, sp.points)) < 0)
                    {
                        std::reverse(polygonWire.begin(), polygonWire.end());
                    }

                    polygon.second.push_back(polygonWire);
                }

                for (auto &point : polygonSharedPosition.points)
                {
                    polygon.first.push_back(point.location3D);
                }

                if (polygon.second.size() > 1)
                {
                    auto comparator = [polygon](std::vector<size_t> a, std::vector<size_t> b)
                    {
                        return WireArea(a, polygon.first) > WireArea(b, polygon.first);
                    };
                    std::sort(polygon.second.begin(), polygon.second.end(), comparator);
                }
            }

            polygons.push_back(polygon);
        }

        return polygons;
    }

    void IfcGeometrySpace::GetSpacesGeomsByBuildingElements(std::vector<BuildingElement> &buildingElements)
    {
        fuzzybools::Geometry unionGeom;
        for (size_t buildingElementId = 0; buildingElementId < buildingElements.size(); buildingElementId++)
        {
            auto buildingElement = buildingElements[buildingElementId];
            if (buildingElement.isVoid)
                continue;

            unionGeom = Union(buildingElement.geometry, unionGeom);
        }

        auto spacesAndBuildings = GetSpacesAndBuildings(unionGeom);

        for (size_t spaceOrBuildingId = 0; spaceOrBuildingId < spacesAndBuildings.size(); spaceOrBuildingId++)
        {
            auto spaceOrBuilding = spacesAndBuildings[spaceOrBuildingId];

            size_t offset = 0;
            std::ofstream ofs("Space_" + std::to_string(spaceOrBuildingId) + ".obj", std::ofstream::out);
            ofs << fuzzybools::ToObj(spaceOrBuilding.geometry, offset);
            ofs.close();
        }

        auto firstLevelBoundaries = GetFirstLevelBoundaries(buildingElements, spacesAndBuildings);

        for (size_t firstLevelBoundaryId = 0; firstLevelBoundaryId < firstLevelBoundaries.size(); firstLevelBoundaryId++)
        {
            auto firstLevelBoundary = firstLevelBoundaries[firstLevelBoundaryId];

            std::cout << firstLevelBoundary.id << ", " << firstLevelBoundary.buildingElement << ", " << firstLevelBoundary.space << std::endl;

            for (auto &polygon : SplitGeometryInPolygons(firstLevelBoundary.geometry))
            {
                std::cout << "polygon" << std::endl;
                for (auto &wire : polygon.second)
                {
                    std::cout << "wire" << std::endl;
                    std::cout << WireArea(wire, polygon.first) << std::endl;
                    for (auto &pointId : wire)
                    {
                        std::cout << "(" << polygon.first[pointId].x << ", " << polygon.first[pointId].y << ", " << polygon.first[pointId].z << ")" << std::endl;
                    }
                }
            }
        }

        auto secondLevelBoundaries = GetSecondLevelBoundaries(buildingElements, spacesAndBuildings, firstLevelBoundaries);

        int secondLevelBoundaryId = 0;
        while (secondLevelBoundaryId < secondLevelBoundaries.size())
        {
            auto secondLevelBoundary = secondLevelBoundaries[secondLevelBoundaryId];

            switch (secondLevelBoundary.boundaryCondition)
            {
            case IfcInternalOrExternalEnum::INTERNAL:
            {
                auto otherSecondLevelBoundary = secondLevelBoundaries[secondLevelBoundaryId + 1];

                std::cout << secondLevelBoundary.id << ", " << secondLevelBoundary.buildingElement << ", " << secondLevelBoundary.space << ", " << secondLevelBoundary.boundaryConditionToString() << std::endl;
                std::cout << otherSecondLevelBoundary.id << ", " << otherSecondLevelBoundary.buildingElement << ", " << otherSecondLevelBoundary.space << ", " << otherSecondLevelBoundary.boundaryConditionToString() << std::endl;

                secondLevelBoundaryId += 2;
                break;
            }
            case IfcInternalOrExternalEnum::EXTERNAL:
            {
                if (spacesAndBuildings[secondLevelBoundary.space].isSpace)
                {
                    std::cout << secondLevelBoundary.id << ", " << secondLevelBoundary.buildingElement << ", " << secondLevelBoundary.space << ", " << secondLevelBoundary.boundaryConditionToString() << std::endl;
                }

                auto otherSecondLevelBoundary = secondLevelBoundaries[secondLevelBoundaryId + 1];
                if (spacesAndBuildings[otherSecondLevelBoundary.space].isSpace)
                {
                    std::cout << otherSecondLevelBoundary.id << ", " << otherSecondLevelBoundary.buildingElement << ", " << otherSecondLevelBoundary.space << ", " << otherSecondLevelBoundary.boundaryConditionToString() << std::endl;
                }

                secondLevelBoundaryId += 2;
                break;
            }
            case IfcInternalOrExternalEnum::NOTDEFINED:
            {
                if (spacesAndBuildings[secondLevelBoundary.space].isSpace)
                {
                    std::cout << secondLevelBoundary.id << ", " << secondLevelBoundary.buildingElement << ", " << secondLevelBoundary.space << ", " << secondLevelBoundary.boundaryConditionToString() << std::endl;
                }

                secondLevelBoundaryId += 1;
                break;
            }
            default:
                secondLevelBoundaryId += 1;
                break;
            }
        }
    }

}