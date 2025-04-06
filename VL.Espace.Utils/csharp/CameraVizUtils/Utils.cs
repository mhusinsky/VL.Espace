using System;
using System.Collections.Generic;
using System.IO;
using System.Text.Json;
using System.Text.Json.Nodes;
using Stride.Core.Mathematics;

public class FrameDataParser
{
    public Dictionary<string, Matrix> Frames { get; private set; } = new Dictionary<string, Matrix>();

    public void LoadFromJson(string filePath)
    {
        if (!File.Exists(filePath))
        {
            throw new FileNotFoundException($"JSON file not found: {filePath}");
        }

        string jsonContent = File.ReadAllText(filePath);
        JsonNode jsonObject = JsonNode.Parse(jsonContent);
        JsonArray framesArray = jsonObject?["frames"]?.AsArray();

        if (framesArray == null)
        {
            throw new Exception("Invalid JSON format: 'frames' key not found");
        }

        foreach (JsonNode frame in framesArray)
        {
            string file = frame?["file_path"]?.GetValue<string>();
            JsonArray matrixArray = frame?["transform_matrix"]?.AsArray();

            if (string.IsNullOrEmpty(file) || matrixArray == null)
            {
                continue;
            }

            Matrix matrix = ConvertToMatrix(matrixArray);
            Frames[file] = matrix;
        }
    }

    private Matrix ConvertToMatrix(JsonArray matrixArray)
    {
        if (matrixArray.Count != 4)
        {
            throw new Exception("Invalid transform_matrix format: Expected 4 rows");
        }

        float[,] values = new float[4, 4];
        for (int i = 0; i < 4; i++)
        {
            JsonArray row = matrixArray[i]?.AsArray();
            if (row == null || row.Count != 4)
            {
                throw new Exception("Invalid transform_matrix format: Each row must have 4 elements");
            }

            for (int j = 0; j < 4; j++)
            {
                values[i, j] = row[j]?.GetValue<float>() ?? throw new Exception("Invalid matrix value");
            }
        }

        return new Matrix(
            values[0, 0], values[0, 1], values[0, 2], values[0, 3],
            values[1, 0], values[1, 1], values[1, 2], values[1, 3],
            values[2, 0], values[2, 1], values[2, 2], values[2, 3],
            values[3, 0], values[3, 1], values[3, 2], values[3, 3]
        );
    }

    public Dictionary<string, Vector3> GetAllCameraPositions()
    {
        Dictionary<string, Vector3> cameraPositions = new Dictionary<string, Vector3>();

        foreach (var frame in Frames)
        {
            Matrix transform = frame.Value;

            // Extract the rotation matrix (top-left 3x3) as a full 4x4 matrix with identity bottom row
            Matrix rotation = new Matrix(
                transform.M11, transform.M12, transform.M13, 0,
                transform.M21, transform.M22, transform.M23, 0,
                transform.M31, transform.M32, transform.M33, 0,
                0, 0, 0, 1
            );

            // Extract the correct translation vector (last column in column-major format)
            Vector4 translation = new Vector4(transform.M14, transform.M24, transform.M34, 1);

            // Compute camera position: -R^T * T (Transpose of rotation times negative translation)
            Vector4 cameraPosition4 = Vector4.Transform(-translation, Matrix.Transpose(rotation));

            // Convert Vector4 to Vector3 (ignore the W component)
            Vector3 cameraPosition = new Vector3(cameraPosition4.X, cameraPosition4.Y, cameraPosition4.Z);

            cameraPositions[frame.Key] = cameraPosition;
        }

        return cameraPositions;
    }
}