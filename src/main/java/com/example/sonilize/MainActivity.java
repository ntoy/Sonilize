package com.example.sonilize;

import android.support.annotation.NonNull;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoErrorException;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoInvalidException;
import com.google.atap.tangoservice.TangoOutOfDateException;
import com.google.atap.tangoservice.TangoPointCloudData;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.PriorityQueue;
import java.util.Vector;

public class MainActivity extends AppCompatActivity {

    private static final float HORIZ_ANGULAR_SPAN = (float) Math.PI;
    private static final float VERT_ANGULAR_SPAN = (float) Math.PI;
    private static final int HORIZ_RES = 64;
    private static final int VERT_RES = 64;
    private static final int MAX_NUM_THINGS = 4;
    private static final float MAX_DISTANCE = 1.5f;
    private static final int MIN_BLOB_SIZE = 50;
    private static final float EPSILON = 0.40f;

    private static final String TAG = MainActivity.class.getSimpleName();

    /* get the audio pan associated with the lateral position of thing */
    private static float panOfThing(Thing thing) {
        return - thing.blob.getAverageTheta() / (HORIZ_ANGULAR_SPAN / 2.0f);
    }

    /* get the audio volume associated with the distance of thing */
    private static float volOfThing(Thing thing) {
        return (float) Math.pow(2.0, -4.0 * thing.blob.getAverageR() / MAX_DISTANCE);
    }

    /* compute a generalized f-mean where transform represents the function f */
    static float generalizedAverage(Iterable<Float> nums, Bijection transform) {
        float sum = 0;
        int size = 0;
        for (float r : nums) {
            sum += transform.fwd(r);
            size++;
        }
        return transform.inv(sum / (float) size);
    }

    /* an invertible function used for generalized f-means */
    private abstract class Bijection {
        abstract float fwd(float x);
        abstract float inv(float y);
    }

    /* a Blob is an aggregate of quantized points in 3-space representing a physical object */
    private class Blob implements Comparable<Blob> {
        private int size;
        private float averageR;
        private float averageTheta;
        private float averagePhi;

        private Blob (Iterable<Triple> points, Bijection transform) {
            size = 0;
            averageTheta = 0.0f;
            averagePhi = 0.0f;
            Vector<Float> rs = new Vector<>();
            for (Triple t : points) {
                size++;
                averageTheta += t.second;
                averagePhi += t.third;
                rs.add(t.first);
            }
            averageR = generalizedAverage(rs, transform);
            averageTheta /= (float) size;
            averagePhi /= (float) size;
        }

        private int getSize() {
            return size;
        }

        private float getAverageR() {
            return averageR;
        }

        private float getAverageTheta() {
            return averageTheta;
        }

        public float getAveragePhi() {
            return averagePhi;
        }

        // compare by radial distance (return positive integer iff this is farther than that)
        public int compareTo(@NonNull Blob that) {
            if (this.averageR < that.averageR) return -1;
            else if (this.averageR == that.averageR) return 0;
            else return 1;
        }

        // the square of the euclidean distance between this and that
        private float distanceSqTo(Blob that) {
            Triple thisCartesian = cartesianOfSpherical(averageR, averageTheta, averagePhi);
            Triple thatCartesian =
                    cartesianOfSpherical(that.averageR, that.averageTheta, that.averagePhi);
            float xDiff = thisCartesian.first - thatCartesian.first;
            float yDiff = thisCartesian.second - thatCartesian.second;
            float zDiff = thisCartesian.third - thatCartesian.third;
            return xDiff * xDiff + yDiff * yDiff + zDiff * zDiff;
        }
    }

    /* three floats */
    private class Triple {
        private float first;
        private float second;
        private float third;

        private Triple(float a, float b, float c) {
            first = a;
            second = b;
            third = c;
        }
    }

    /* a blob and its associated sound */
    private class Thing {
        private Blob blob;
        private int streamID;

        private Thing(Blob blob, int streamID) {
            this.blob = blob;
            this.streamID = streamID;
        }
    }

    /* get spherical coordinates of (x, y, z) as (r, theta, phi) where r is the
     * radial distance, theta is longitude (the signed angle measured clockwise
     * from the z axis in the xz-plane), and phi is the latitude (the signed
     * angle measured upwards from the xz plane)
     */
    private Triple sphericalOfCartesian(float x, float y, float z){
        float r = (float) Math.sqrt(x*x + y*y + z*z);
        float theta = (float) Math.atan(x / z);
        float phi = (float) Math.asin(-y / r);
        return new Triple(r, theta, phi);
    }

    /* get cartesian coordinates of (r, theta, phi) as (x, y, z) where r is the
     * radial distance, theta is longitude (the signed angle measured clockwise
     * from the z axis in the xz-plane), and phi is the latitude (the signed
     * angle measured upwards from the xz plane)
     */
    private Triple cartesianOfSpherical(float r, float theta, float phi) {
        float y = - r * (float) Math.sin(phi);
        float x = r * (float) (Math.cos(phi) * Math.sin(theta));
        float z = x / (float) Math.tan(theta);
        return new Triple(x, y, z);
    }

    private Vector<Thing> things;
    private float[][] depthMatrix;
    private int[][] counts;
    private Tango mTango;
    private TangoConfig mConfig;
    private Bijection reciprocal;
    private LoopedSoundCollection soundCollection;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        mTango = new Tango(MainActivity.this);
        reciprocal = new Bijection() {
            @Override
            float fwd(float x) {
                return 1.0f / x;
            }
            @Override
            float inv(float y) {
                return 1.0f / y;
            }
        };
    }

    @Override
    protected void onResume() {
        super.onResume();

        things = new Vector<>();
        counts = new int[HORIZ_RES][VERT_RES];
        depthMatrix = new float[HORIZ_RES][VERT_RES];
        int[] resids = {R.raw.sound_1, R.raw.sound_2, R.raw.sound_3, R.raw.sound_4,
                        R.raw.sound_5, R.raw.sound_6, R.raw.sound_7, R.raw.sound_8};
        soundCollection = new LoopedSoundCollection(MainActivity.this, resids, MAX_NUM_THINGS);

        mTango = new Tango(MainActivity.this, new Runnable() {
            @Override
            public void run() {
                synchronized (MainActivity.this) {
                    try {
                        mConfig = setupTangoConfig(mTango);
                        mTango.connect(mConfig);
                        startupTango();
                    } catch (TangoOutOfDateException e) {
                        Log.e(TAG, getString(R.string.exception_out_of_date), e);
                    } catch (TangoErrorException e) {
                        Log.e(TAG, getString(R.string.exception_tango_error), e);
                    } catch (TangoInvalidException e) {
                        Log.e(TAG, getString(R.string.exception_tango_invalid), e);
                    }
                }
            }
        });
    }

    @Override
    protected void onPause() {
        super.onPause();
        synchronized (this) {
            try {
                mTango.disconnect();
            } catch (TangoErrorException e) {
                Log.e(TAG, getString(R.string.exception_tango_error), e);
            }
        }
    }

    private TangoConfig setupTangoConfig(Tango tango) {
        // Create a new Tango Configuration and enable the Depth Sensing API.
        TangoConfig config = tango.getConfig(TangoConfig.CONFIG_TYPE_DEFAULT);
        config.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true);
        config.putInt(TangoConfig.KEY_INT_DEPTH_MODE, TangoConfig.TANGO_DEPTH_MODE_POINT_CLOUD);
        return config;
    }

    private void startupTango() {
        // Lock configuration and connect to Tango.
        // Select coordinate frame pair.
        final ArrayList<TangoCoordinateFramePair> framePairs =
                new ArrayList<>();
        framePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE));

        // Listen for new Tango data
        mTango.connectListener(framePairs, new Tango.OnTangoUpdateListener() {
            @Override
            public void onPoseAvailable(final TangoPoseData pose) {
                // not using TangoPoseData
            }

            @Override
            public void onXyzIjAvailable(TangoXyzIjData xyzIj) {
                // not using onXyzIjAvailable
            }

            @Override
            public void onPointCloudAvailable(final TangoPointCloudData pointCloudData) {
                handlePointCloud(pointCloudData);
            }

            @Override
            public void onTangoEvent(final TangoEvent event) {
                // ignoring TangoEvents.
            }

            @Override
            public void onFrameAvailable(int cameraId) {
                // not using onFrameAvailable
            }
        });
    }

    /* the action we take for each new available point cloud */
    private void handlePointCloud(TangoPointCloudData pointCloudData) {
        Log.i(TAG, "IDs");
        for (Thing t : things) {
            Log.i(TAG, Integer.toString(t.streamID));
        }
        quantize(pointCloudData.points, pointCloudData.numPoints, reciprocal);
        Vector<Blob> blobs = findBlobs(depthMatrix, reciprocal, MAX_DISTANCE,
                                        MIN_BLOB_SIZE, MAX_DISTANCE, MAX_NUM_THINGS);
        updateThings(blobs, EPSILON);
    }

    /* update all the private things by tracking blobs */
    private void updateThings(Iterable<Blob> newBlobs, float epsilon) {
        // update the blobs
        Vector<Thing> newThings = new Vector<>(MAX_NUM_THINGS);

        // attempt to match newly detected blob with a perviously detected one
        for (Blob b : newBlobs) {
            float distSq = Float.POSITIVE_INFINITY;
            int index = -1;
            // for a given new blob, find its nearest neighbor from the previous point cloud
            for (int i = 0; i < things.size(); i++) {
                float d = things.elementAt(i).blob.distanceSqTo(b);
                if (d < distSq) {
                    distSq = d;
                    index = i;
                }
            }
            if (distSq <= epsilon * epsilon) {
                // if nearest neighbor is close enough, both blobs are deemed to be the same
                // object at two points in time, old thing is updated with new blob's
                // characteristics, and old sound is inherited
                Thing t = things.remove(index);
                t.blob = b;
                soundCollection.setVolPan(t.streamID, volOfThing(t), panOfThing(t));
                newThings.add(t);
            }
            else {
                // if deemed new, create a new thing (with new sound)
                Thing t = new Thing(b, soundCollection.activateLeastRecent());
                soundCollection.setVolPan(t.streamID, volOfThing(t), panOfThing(t));
                soundCollection.play(t.streamID);
                newThings.add(t);
            }
        }

        // discard things that have disappeared from the visual field
        for (Thing t : things) {
            soundCollection.pause(t.streamID);
            soundCollection.deactivate(t.streamID);
        }

        things = newThings;
    }

    /* find all Blobs made up of at least minSize grid cells, each cell closer than maxCellDist,
     * such that the averaged distance (f-mean relative to transform) of the blob is at most
     * maxBlobDist; output at most maxNum blobs, prioritized by smallest distance */
    private Vector<Blob> findBlobs(float[][] grid, Bijection transform, float maxCellDist,
                                   int minSize, float maxBlobDist, int maxNum) {
        boolean[][] marked = new boolean[grid.length][grid[0].length];
        PriorityQueue<Blob> pq = new PriorityQueue<>();
        for (int i = 0; i < grid.length; i++) {
            for (int j = 0; j < grid[0].length; j++) {
                if (!marked[i][j] && grid[i][j] <= maxCellDist) {
                    Vector<Triple> preBlob = new Vector<>();
                    dfs(grid, marked, preBlob, maxCellDist, i, j);
                    Blob blob = new Blob(preBlob, transform);
                    if (blob.getSize() >= minSize && blob.getAverageR() <= maxBlobDist)
                        pq.add(blob);
                }
            }
        }
        Vector<Blob> result = new Vector<>();
        for (int i = 0; i < maxNum; i++) {
            if (!pq.isEmpty())
                result.add(pq.poll());
            else break;
        }
        return result;
    }

    /* find the component containing cell (i, j) of grid consisting of all cells closer than
       maxCellDist; result stored in points */
    private void dfs(float[][] grid, boolean[][] marked, Vector<Triple> points,
                     float maxCellDist, int i, int j) {
        if (i < 0 || i >= grid.length || j < 0 || j >= grid[0].length)
            return;
        if (!marked[i][j] && grid[i][j] <= maxCellDist) {
            marked[i][j] = true;
            float theta = (float) i * HORIZ_ANGULAR_SPAN / HORIZ_RES - HORIZ_ANGULAR_SPAN / 2.0f;
            float phi = (float) j * VERT_ANGULAR_SPAN / VERT_RES - VERT_ANGULAR_SPAN / 2.0f;
            points.add(new Triple(grid[i][j], theta, phi));
            dfs(grid, marked, points, maxCellDist, i + 1, j);
            dfs(grid, marked, points, maxCellDist, i, j + 1);
            dfs(grid, marked, points, maxCellDist, i - 1, j);
            dfs(grid, marked, points, maxCellDist, i, j - 1);
        }
        else {
            marked[i][j] = true;
        }
    }

    /* compute a quantized version of pointCloudBuffer consisting of a grid where each cell
       represents a 2D range of viewing angles, and the value in each cell corresponds to the
       averaged radial distance of all 3D points lying in that range */
    private void quantize(FloatBuffer pointCloudBuffer, int numPoints, Bijection transform) {
        for (int i = 0; i < HORIZ_RES; i++) {
            for (int j = 0; j < VERT_RES; j++) {
                counts[i][j] = 0;
                depthMatrix[i][j] = 0.0f;
            }
        }
        for (int i = 0; i < 4 * numPoints; i += 4) {
            float x = pointCloudBuffer.get(i),
                    y = pointCloudBuffer.get(i+1),
                    z = pointCloudBuffer.get(i+2);

            Triple sphCoord = sphericalOfCartesian(x, y, z);
            int col = (int) ((sphCoord.second + HORIZ_ANGULAR_SPAN / 2.0f)
                    * (float) HORIZ_RES / HORIZ_ANGULAR_SPAN);
            if (col < 0 || col >= HORIZ_RES)
                throw new RuntimeException("Debug: col = " + col);
            int row = (int) ((sphCoord.third + VERT_ANGULAR_SPAN / 2.0f)
                    * (float) VERT_RES / VERT_ANGULAR_SPAN);
            if (row < 0 || row >= HORIZ_RES)
                throw new RuntimeException("Debug: row = " + row);
            depthMatrix[col][row] += transform.fwd(sphCoord.first);
            counts[col][row]++;
        }
        for (int i = 0; i < HORIZ_RES; i++) {
            for (int j = 0; j < VERT_RES; j++) {
                if (counts[i][j] == 0)
                    depthMatrix[i][j] = Float.POSITIVE_INFINITY;
                else
                    depthMatrix[i][j] = transform.inv(depthMatrix[i][j] / (float) counts[i][j]);
            }
        }
    }
}