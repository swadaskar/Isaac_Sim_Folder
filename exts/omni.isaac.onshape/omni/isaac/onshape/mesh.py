# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np


class Mesh:
    def __init__(self, vertices, verticesNormals, verticesUVs, faceVertexCount, facesIndices, facetsPerColor, colors):
        self.vertices = vertices
        self.vertices_normals = verticesNormals
        self.vertices_UVs = verticesUVs
        self.face_vertex_count = faceVertexCount
        self.face_indices = facesIndices
        self.facets_per_color = facetsPerColor
        self.colors = colors

    def get_extent(self):
        extent = np.max(self.vertices, axis=0) - np.min(self.vertices, axis=0)
        return extent

    @staticmethod
    def GetFromPartSpec(part):
        bodies = part["bodies"]
        body_color = [b["color"] for b in bodies]
        faces = [(b, s["color"]) for s in bodies for b in s["faces"]]
        faces_color = [f[0]["color"] if "color" in f[0].keys() else f[1] for f in faces]
        colors_unique = set([f for f in faces_color])
        facets = {
            c: [a for s in faces for a in s[0]["facets"] if (s[0]["color"] if "color" in s[0].keys() else s[1]) == c]
            for c in colors_unique
        }
        facets_per_color = [len(facets[c]) for c in facets.keys()]
        for i in range(len(facets_per_color)):
            start = 0 if i == 0 else facets_per_color[i - 1][-1] + 1
            facets_per_color[i] = list(range(start, start + facets_per_color[i]))
        vertices = np.array(part["facetPoints"])
        indices_per_facet = [i["indices"] for c in facets.keys() for i in facets[c]]
        indices = np.array([i for ids in indices_per_facet for i in ids])

        verticesNormals_per_facet = [i["vertexNormals"] for c in facets.keys() for i in facets[c]]
        verticesNormals = np.array([i for ids in verticesNormals_per_facet for i in ids])

        verticesUVs_per_facet = [i["textureCoordinates"] for c in facets.keys() for i in facets[c]]
        verticesUVs = np.array([i for ids in verticesUVs_per_facet for i in ids])

        facetVertexCount = np.array([len(i) for i in indices_per_facet])
        return Mesh(vertices, verticesNormals, verticesUVs, facetVertexCount, indices, facets_per_color, facets.keys())
