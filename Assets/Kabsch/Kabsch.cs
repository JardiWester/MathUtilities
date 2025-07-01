using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Profiling;


[ExecuteInEditMode]
public class KabschInOneClass : MonoBehaviour
{
    public Transform[] editorPoints = new Transform[0];
    public Transform[] realLifeAnchorPoints = new Transform[0];
    Vector3[] points; Vector4[] refPoints;

    public bool useRealLifeAnchors;
    public bool updateEveryTick;
    public bool moveAnchors;

    public KabschManager kabschManager;

    //KabschSolver solver = new KabschSolver();

    void Start()
    {
        Initialize();

        if (Application.isPlaying)
        {
            updateEveryTick = true;
            useRealLifeAnchors = true;
            moveAnchors = true;
        }
        else
        {
            updateEveryTick = false;
        }
    }

    //Set up the Input Points
    void Initialize()
    {
        points = new Vector3[editorPoints.Length];
        refPoints = new Vector4[editorPoints.Length];
        for (int i = 0; i < editorPoints.Length; i++)
        {
            points[i] = editorPoints[i].position;
        }
    }

    //Calculate the Kabsch Transform and Apply it to the input points
    void Update()
    {
        if (updateEveryTick && ControllerTrackingThing.rightController.activeSelf == true && kabschManager.realLifeAnchorIDs.Count >= kabschManager.editorAnchorIDs.Count)
        {
            UpdateKabschPositioning();
        }
    }

    public void UpdateKabschPositioning(bool callibration = false)
    {
        if (editorPoints.Length == 0 || (realLifeAnchorPoints.Length == 0 && callibration == false))
        {
            return;
        }

        if (editorPoints.Length > points.Length || editorPoints.Length < points.Length)
        {
            Initialize();
        }


        if (useRealLifeAnchors && !callibration)
        {
            for (int i = 0; i < realLifeAnchorPoints.Length; i++)
            {
                refPoints[i] = new Vector4(realLifeAnchorPoints[i].position.x, realLifeAnchorPoints[i].position.y, realLifeAnchorPoints[i].position.z, realLifeAnchorPoints[i].localScale.x);
            }
        }
        else
        {
            for (int i = 0; i < editorPoints.Length; i++)
            {
                refPoints[i] = new Vector4(editorPoints[i].position.x, editorPoints[i].position.y, editorPoints[i].position.z, editorPoints[i].localScale.x);
            }
        }

        Matrix4x4 kabschTransform = SolveKabsch(points, refPoints);


        if (moveAnchors)
        {
            for (int i = 0; i < editorPoints.Length; i++)
            {
                editorPoints[i].position = kabschTransform.MultiplyPoint3x4(points[i]);
            }

            if (!callibration)
            {
                kabschManager.avaragePositionScript.MoveIntoPosition();

            }
        }
    }

    Vector3[] QuatBasis = new Vector3[3];
    Vector3[] DataCovariance = new Vector3[3];
    Quaternion OptimalRotation = Quaternion.identity;

    public Vector3 refCentroidTemptest;
    public Quaternion inCentroidTemptest;


    public float scaleRatio = 1f;
    public Matrix4x4 SolveKabsch(Vector3[] inPoints, Vector4[] refPoints, bool solveRotation = true, bool solveScale = false)
    {
        if (inPoints.Length != refPoints.Length) { return Matrix4x4.identity; }

        //Calculate the centroid offset and construct the centroid-shifted point matrices
        Vector3 inCentroid = Vector3.zero; Vector3 refCentroid = Vector3.zero;
        float inTotal = 0f, refTotal = 0f;
        for (int i = 0; i < inPoints.Length; i++)
        {
            inCentroid += new Vector3(inPoints[i].x, inPoints[i].y, inPoints[i].z) * refPoints[i].w;
            inTotal += refPoints[i].w;
            refCentroid += new Vector3(refPoints[i].x, refPoints[i].y, refPoints[i].z) * refPoints[i].w;
            refTotal += refPoints[i].w;
        }
        inCentroid /= inTotal;
        refCentroid /= refTotal;

        refCentroidTemptest = refCentroid;

        //Calculate the scale ratio
        if (solveScale)
        {
            float inScale = 0f, refScale = 0f;
            for (int i = 0; i < inPoints.Length; i++)
            {
                inScale += (new Vector3(inPoints[i].x, inPoints[i].y, inPoints[i].z) - inCentroid).magnitude;
                refScale += (new Vector3(refPoints[i].x, refPoints[i].y, refPoints[i].z) - refCentroid).magnitude;
            }
            scaleRatio = (refScale / inScale);
        }

        //Calculate the 3x3 covariance matrix, and the optimal rotation
        if (solveRotation)
        {
            Profiler.BeginSample("Solve Optimal Rotation");
            extractRotation(TransposeMultSubtract(inPoints, refPoints, inCentroid, refCentroid, DataCovariance), ref OptimalRotation);
            Profiler.EndSample();
        }

        return Matrix4x4.TRS(refCentroid, Quaternion.identity, Vector3.one) *
               Matrix4x4.TRS(Vector3.zero, OptimalRotation, Vector3.one) *
               Matrix4x4.TRS(-inCentroid, Quaternion.identity, Vector3.one);
    }

    //https://animation.rwth-aachen.de/media/papers/2016-MIG-StableRotation.pdf
    //Iteratively apply torque to the basis using Cross products (in place of SVD)
    void extractRotation(Vector3[] A, ref Quaternion q)
    {
        for (int iter = 0; iter < 9; iter++)
        {
            Profiler.BeginSample("Iterate Quaternion");
            q.FillMatrixFromQuaternionWithEditor(ref QuatBasis);
            Vector3 omega = (Vector3.Cross(QuatBasis[0], A[0]) +
                             Vector3.Cross(QuatBasis[1], A[1]) +
                             Vector3.Cross(QuatBasis[2], A[2])) *
             (1f / Mathf.Abs(Vector3.Dot(QuatBasis[0], A[0]) +
                             Vector3.Dot(QuatBasis[1], A[1]) +
                             Vector3.Dot(QuatBasis[2], A[2]) + 0.000000001f));

            float w = omega.magnitude;
            if (w < 0.000000001f)
                break;
            q = Quaternion.AngleAxis(w * Mathf.Rad2Deg, omega / w) * q;
            q = Quaternion.Lerp(q, q, 0f); //Normalizes the Quaternion; critical for error suppression
            inCentroidTemptest = q;
            Profiler.EndSample();
        }
    }

    //Calculate Covariance Matrices --------------------------------------------------
    public static Vector3[] TransposeMultSubtract(Vector3[] vec1, Vector4[] vec2, Vector3 vec1Centroid, Vector3 vec2Centroid, Vector3[] covariance)
    {
        Profiler.BeginSample("Calculate Covariance Matrix");
        for (int i = 0; i < 3; i++)
        { //i is the row in this matrix
            covariance[i] = Vector3.zero;
        }

        for (int k = 0; k < vec1.Length; k++)
        {//k is the column in this matrix
            Vector3 left = (vec1[k] - vec1Centroid) * vec2[k].w;
            Vector3 right = (new Vector3(vec2[k].x, vec2[k].y, vec2[k].z) - vec2Centroid) * Mathf.Abs(vec2[k].w);

            covariance[0][0] += left[0] * right[0];
            covariance[1][0] += left[1] * right[0];
            covariance[2][0] += left[2] * right[0];
            covariance[0][1] += left[0] * right[1];
            covariance[1][1] += left[1] * right[1];
            covariance[2][1] += left[2] * right[1];
            covariance[0][2] += left[0] * right[2];
            covariance[1][2] += left[1] * right[2];
            covariance[2][2] += left[2] * right[2];
        }

        Profiler.EndSample();
        return covariance;
    }
    public static Vector3[] TransposeMultSubtract(Vector3[] vec1, Vector3[] vec2, ref Vector3[] covariance)
    {
        for (int i = 0; i < 3; i++) covariance[i] = Vector3.zero;

        for (int k = 0; k < vec1.Length; k++)
        {//k is the column in this matrix
            Vector3 left = vec1[k];
            Vector3 right = vec2[k];

            covariance[0][0] += left[0] * right[0];
            covariance[1][0] += left[1] * right[0];
            covariance[2][0] += left[2] * right[0];
            covariance[0][1] += left[0] * right[1];
            covariance[1][1] += left[1] * right[1];
            covariance[2][1] += left[2] * right[1];
            covariance[0][2] += left[0] * right[2];
            covariance[1][2] += left[1] * right[2];
            covariance[2][2] += left[2] * right[2];
        }
        return covariance;
    }


}