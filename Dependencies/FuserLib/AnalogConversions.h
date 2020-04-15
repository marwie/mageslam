//
// Copyright (C) Microsoft Corporation. All rights reserved.
//
#pragma once
#include <MathLib\SO3.h>
#include <MathLib\SE3.h>
#include <MathLib\Matrix.h>
#include <array>
#include <assert.h>
#include <arcana/macros.h>

namespace mage
{
    inline std::array<float, 3 * 3> Native3x3ToArray3x3(const double inRotVals[3 * 3])
    {
        // returns world to imu rotation matrix
        // row major storage convention  row0col0, row0col1, row0col2, row1col0 ...
        // column vector math convention result = mat * vec NOT result = vec * mat

        return{ (float)inRotVals[0], (float)inRotVals[1], (float)inRotVals[2],
                (float)inRotVals[3], (float)inRotVals[4], (float)inRotVals[5],
                (float)inRotVals[6], (float)inRotVals[7], (float)inRotVals[8] };
    }

    inline std::array<float, 4 * 4> Native3x3ToArray4x4(const double inRotVals[3 * 3])
    {
        // returns world to imu rotation matrix
        // row major storage convention  row0col0, row0col1, row0col2, row1col0 ...
        // column vector math convention result = mat * vec NOT result = vec * mat

        return{ (float)inRotVals[0], (float)inRotVals[1], (float)inRotVals[2], 0,
                (float)inRotVals[3], (float)inRotVals[4], (float)inRotVals[5], 0,
                (float)inRotVals[6], (float)inRotVals[7], (float)inRotVals[8], 0,
                                  0,                   0,                   0, 1 };
    }

    inline std::array<float, 3 * 3> AnalogSO3ToArray3x3(const ST::SO3<double>& inRotation)
    {
        return Native3x3ToArray3x3(inRotation.R);
    }

    inline std::array<float, 4 * 4> AnalogSO3ToArray4x4(const ST::SO3<double>& inRotation)
    {
        return Native3x3ToArray4x4(inRotation.R);
    }

    inline std::array<float, 4 * 4> AnalogSE3ToMat4x4(const ST::SE3<double>& inPose)
    {
        assert(ST::SE3_is_good(inPose));

        // does the coerce to make sure it is a good rotation
        ST::SO3<double> inSO3;
        ST::SE3_get_SO3<double, double>(inSO3, inPose);
        std::array<float, 4 * 4> arrayMat = { (float)inSO3.R[0], (float)inSO3.R[1], (float)inSO3.R[2], (float)inPose.t[0],
                                                 (float)inSO3.R[3], (float)inSO3.R[4], (float)inSO3.R[5], (float)inPose.t[1],
                                                 (float)inSO3.R[6], (float)inSO3.R[7], (float)inSO3.R[8], (float)inPose.t[2],
                                                    0.0f,        0.0f,        0.0f,              1.0f };

        return arrayMat;
    }

    inline std::array<double, 6 * 6> AnalogMat6x6ToArray(const ST::Matrix<6, 6, double>& inMat)
    {
        return { inMat(0,0), inMat(0,1), inMat(0,2), inMat(0,3), inMat(0,4), inMat(0,5),
                    inMat(1,0), inMat(1,1), inMat(1,2), inMat(1,3), inMat(1,4), inMat(1,5),
                    inMat(2,0), inMat(2,1), inMat(2,2), inMat(2,3), inMat(2,4), inMat(2,5),
                    inMat(3,0), inMat(3,1), inMat(3,2), inMat(3,3), inMat(3,4), inMat(3,5),
                    inMat(4,0), inMat(4,1), inMat(4,2), inMat(4,3), inMat(4,4), inMat(4,5),
                    inMat(5,0), inMat(5,1), inMat(5,2), inMat(5,3), inMat(5,4), inMat(5,5)
        };
    }

    // row major storage convention  row0col0, row0col1, row0col2, row1col0 ...
    // column major vector math convention result = mat * vec NOT result = vec * mat

    inline ST::SO3<double> Array3x3ToAnalogSO3(const std::array<float, 3 * 3>& inMat)
    {
        ST::SO3<double> dstSO3;

        bool success = ST::SO3_copy_from_array<double, float>(dstSO3, inMat.data());
        assert(success && "invalid so3");
        UNUSED(success);

        return dstSO3;
    }

    inline ST::SE3<double> Array4x4ToAnalogSE3(const std::array<float, 4 * 4>& inMat)
    {
        // row major storage convention  row0col0, row0col1, row0col2, row1col0 ...
        // column major vector math convention result = mat * vec NOT result = vec * mat
        ST::SE3<double> dstSE3;

        dstSE3.R[0] = inMat[0];  dstSE3.R[1] = inMat[1]; dstSE3.R[2] = inMat[2];
        dstSE3.R[3] = inMat[4];  dstSE3.R[4] = inMat[5]; dstSE3.R[5] = inMat[6];
        dstSE3.R[6] = inMat[8];  dstSE3.R[7] = inMat[9]; dstSE3.R[8] = inMat[10];

        ST::SO3_coerce(dstSE3.R);

        dstSE3.t[0] = inMat[3];  dstSE3.t[1] = inMat[7]; dstSE3.t[2] = inMat[11];

        return dstSE3;
    }

    inline ST::Matrix<6, 6, double> Array6x6ToAnalogMat(const std::array<double, 6 * 6>& inArray)
    {
        ST::Matrix < 6, 6, double > outMat;
        for (int idxRow = 0; idxRow < 6; idxRow++)
        {
            for (int idxCol = 0; idxCol < 6; idxCol++)
            {
                outMat(idxRow, idxCol) = inArray[idxRow * 6 + idxCol];
            }
        }
        return outMat;
    }

    inline ST::Matrix<3, 3, double> Array3x3ToAnalogMat(const std::array<float, 3 * 3>& inArray)
    {
        ST::Matrix < 3, 3, double > outMat;
        for (int idxRow = 0; idxRow < 3; idxRow++)
        {
            for (int idxCol = 0; idxCol < 3; idxCol++)
            {
                outMat(idxRow, idxCol) = inArray[idxRow * 3 + idxCol];
            }
        }

        return outMat;
    }

    inline ST::Matrix<3, 3, double> Array3x3ToAnalogMat(const std::array<double, 3 * 3>& inArray)
    {
        ST::Matrix < 3, 3, double > outMat;
        for (int idxRow = 0; idxRow < 3; idxRow++)
        {
            for (int idxCol = 0; idxCol < 3; idxCol++)
            {
                outMat(idxRow, idxCol) = inArray[idxRow * 3 + idxCol];
            }
        }

        return outMat;
    }
}
