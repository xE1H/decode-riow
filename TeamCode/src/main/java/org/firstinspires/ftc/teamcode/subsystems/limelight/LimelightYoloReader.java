package org.firstinspires.ftc.teamcode.subsystems.limelight;

import android.os.health.SystemHealthManager;

import com.acmerobotics.dashboard.config.Config;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@Config
public class LimelightYoloReader {
    public static String baseUrl = "http://172.29.0.1:8000";
    public static Map<Integer, Limelight.Sample.Color> clsMap = Map.of(
            0, Limelight.Sample.Color.BLUE,
            1, Limelight.Sample.Color.RED,
            2, Limelight.Sample.Color.YELLOW
    );

    public LimelightYoloReader() {

    }

    // todo getDetections
    public void getDetections() {
        JSONObject json = sendGetRequest("/detection_boxes.json");
        if (json == null) {
            System.out.println("Failed to get detection boxes: JSON null");
            return;
        }

        List<Limelight.Sample> samples = new ArrayList<>();

        try {
            JSONArray detectionsArray = json.getJSONArray("detections");

            for (int i = 0; i < detectionsArray.length(); i++) {
                JSONObject detectionObj = detectionsArray.getJSONObject(i);

                int classId = detectionObj.getInt("class");
                int x1 = detectionObj.getInt("x1");
                int y1 = detectionObj.getInt("y1");
                int x2 = detectionObj.getInt("x2");
                int y2 = detectionObj.getInt("y2");
                System.out.println("Detected sample " + classId + ", (" + x1 + ", " + y1 + "), (" + x2 + ", " + y2 + ")");
                //samples.add(new Detection(classId, x1, y1, x2, y2));
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

    }

    private JSONObject sendGetRequest(String endpoint) {
        HttpURLConnection connection = null;
        try {
            String urlString = baseUrl + endpoint;
            URL url = new URL(urlString);
            connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");
            connection.setReadTimeout(100);
            connection.setConnectTimeout(100);

            int responseCode = connection.getResponseCode();
            if (responseCode == HttpURLConnection.HTTP_OK) {
                String response = readResponse(connection);
                System.out.println("GET " + endpoint);
                System.out.println(response);
                return new JSONObject(response);
            } else {
                System.out.println("GET Error: " + responseCode);
            }
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            if (connection != null) {
                connection.disconnect();
            }
        }
        return null;
    }

    private String readResponse(HttpURLConnection connection) throws IOException {
        BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
        StringBuilder response = new StringBuilder();
        String line;

        while ((line = reader.readLine()) != null) {
            response.append(line);
        }
        reader.close();

        return response.toString();
    }

}
