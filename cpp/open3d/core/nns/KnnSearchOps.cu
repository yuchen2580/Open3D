// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018-2021 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include "open3d/core/Tensor.h"
#include "open3d/core/nns/FixedRadiusIndex.h"
#include "open3d/core/nns/KnnIndex.h"
#include "open3d/core/nns/KnnSearchImpl.cuh"

namespace open3d {
namespace core {
namespace nns {

template <class T>
void KnnSearchCUDA(const Tensor& points,
                   const Tensor& points_row_splits,
                   const Tensor& queries,
                   const Tensor& queries_row_splits,
                   int knn,
                   Tensor& neighbors_index,
                   Tensor& neighbors_distance) {
    const cudaStream_t stream = 0;

    Device device = points.GetDevice();
    NeighborSearchAllocator<T> output_allocator(device);

    int num_points = points.GetShape()[0];
    int num_queries = queries.GetShape()[0];
    knn = num_points > knn ? knn : num_points;

    open3d::core::nns::impl::KnnSearchCUDA(
            stream, points.GetShape()[0], points.GetDataPtr<T>(),
            queries.GetShape()[0], queries.GetDataPtr<T>(),
            points_row_splits.GetShape()[0],
            points_row_splits.GetDataPtr<int64_t>(),
            queries_row_splits.GetShape()[0],
            queries_row_splits.GetDataPtr<int64_t>(), knn, output_allocator);

    neighbors_index =
            output_allocator.NeighborsIndex().View({num_queries, knn});
    neighbors_distance =
            output_allocator.NeighborsDistance().View({num_queries, knn});
}

#define INSTANTIATE(T)                                                        \
    template void KnnSearchCUDA<T>(                                           \
            const Tensor& points, const Tensor& points_row_splits,            \
            const Tensor& queries, const Tensor& queries_row_splits, int knn, \
            Tensor& neighbors_index, Tensor& neighbors_distance);

INSTANTIATE(float)
INSTANTIATE(double)
}  // namespace nns
}  // namespace core
}  // namespace open3d
