/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#pragma once

#include <glm/glm.hpp>

#include "util.h"
#include "is-inside-mesh.h"
#include "geometry.h"
#include "bvh.h"

namespace fuzzybools
{
    enum class BooleanOperator
    {
        UNION,
        INTERSECTION,
        DIFFERENCE
    };

    static Geometry clipBooleanResult(const BooleanOperator op, Geometry &mesh, BVH &bvh1, BVH &bvh2)
    {
        Geometry booleanResult;

        for (uint32_t i = 0; i < mesh.numFaces; i++)
        {
            Face tri = mesh.GetFace(i);
            glm::dvec3 a = mesh.GetPoint(tri.i0);
            glm::dvec3 b = mesh.GetPoint(tri.i1);
            glm::dvec3 c = mesh.GetPoint(tri.i2);

            glm::dvec3 triCenter = (a + b + c) * 1.0 / 3.0;
            glm::dvec3 n = computeNormal(a, b, c);
            auto isInside1Result = isInsideMesh(triCenter, n, *bvh1.ptr, bvh1);
            auto isInside2Result = isInsideMesh(triCenter, n, *bvh2.ptr, bvh2);

            if (isInside1Result.loc != MeshLocation::BOUNDARY && isInside2Result.loc != MeshLocation::BOUNDARY)
            {
                // neither boundary, no dice
            }
            else if (isInside1Result.loc == MeshLocation::BOUNDARY && isInside2Result.loc == MeshLocation::BOUNDARY)
            {
                auto dot = glm::dot(isInside1Result.normal, isInside2Result.normal);

                if (dot > 0 && op != BooleanOperator::DIFFERENCE)
                {
                    booleanResult.AddFace(a, b, c);
                }
                if (dot < 0 && op == BooleanOperator::DIFFERENCE)
                {
                    booleanResult.AddFace(a, b, c);
                }
            }
            else if (isInside1Result.loc == MeshLocation::INSIDE)
            {
                if (op != BooleanOperator::UNION)
                {
                    if (glm::dot(n, isInside2Result.normal) < 0)
                    {
                        booleanResult.AddFace(a, b, c);
                    }
                    else
                    {
                        booleanResult.AddFace(b, a, c);
                    }
                }
            }
            else if (isInside1Result.loc == MeshLocation::OUTSIDE)
            {
                if (op == BooleanOperator::UNION)
                {
                    if (glm::dot(n, isInside2Result.normal) < 0)
                    {
                        booleanResult.AddFace(b, a, c);
                    }
                    else
                    {
                        booleanResult.AddFace(a, b, c);
                    }
                }
            }
            else if (isInside2Result.loc == MeshLocation::INSIDE)
            {
                if (op == BooleanOperator::INTERSECTION)
                {
                    if (glm::dot(n, isInside1Result.normal) < 0)
                    {
                        booleanResult.AddFace(b, a, c);
                    }
                    else
                    {
                        booleanResult.AddFace(a, b, c);
                    }
                }
            }
            else if (isInside2Result.loc == MeshLocation::OUTSIDE)
            {
                if (op != BooleanOperator::INTERSECTION)
                {
                    if (glm::dot(n, isInside1Result.normal) < 0)
                    {
                        booleanResult.AddFace(b, a, c);
                    }
                    else
                    {
                        booleanResult.AddFace(a, b, c);
                    }
                }
            }
        }

        return booleanResult;
    }

    static std::pair<Geometry, std::pair<Geometry, Geometry>> SplitFirstBoundary(Geometry &mesh, BVH &bvh1, BVH &bvh2)
    {
        std::pair<Geometry, std::pair<Geometry, Geometry>> result;

        for (uint32_t i = 0; i < mesh.numFaces; i++)
        {
            Face tri = mesh.GetFace(i);
            glm::dvec3 a = mesh.GetPoint(tri.i0);
            glm::dvec3 b = mesh.GetPoint(tri.i1);
            glm::dvec3 c = mesh.GetPoint(tri.i2);

            glm::dvec3 triCenter = (a + b + c) * 1.0 / 3.0;
            glm::dvec3 n = computeNormal(a, b, c);
            auto isInside1Result = isInsideMesh(triCenter, n, *bvh1.ptr, bvh1);
            auto isInside2Result = isInsideMesh(triCenter, n, *bvh2.ptr, bvh2);

            if (isInside1Result.loc == MeshLocation::BOUNDARY)
            {
                auto dot = glm::dot(n, isInside1Result.normal);

                switch (isInside2Result.loc)
                {
                case MeshLocation::BOUNDARY:
                {
                    if (dot < 0)
                    {
                        result.first.AddFace(b, a, c);
                    }
                    else
                    {
                        result.first.AddFace(a, b, c);
                    }
                    break;
                }
                case MeshLocation::OUTSIDE:
                {
                    if (dot < 0)
                    {
                        result.second.first.AddFace(b, a, c);
                    }
                    else
                    {
                        result.second.first.AddFace(a, b, c);
                    }
                    break;
                }
                case MeshLocation::INSIDE:
                {
                    if (dot < 0)
                    {
                        result.second.second.AddFace(b, a, c);
                    }
                    else
                    {
                        result.second.second.AddFace(a, b, c);
                    }
                }
                default:
                    break;
                }
            }
        }

        return result;
    }
}