package com.example.sonilize;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;

public class TestClient extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_test_client);
    }

    @Override
    protected void onResume() {
        super.onResume();

        Thread thread = new Thread() {
            public void run() {
                int id = 0;
                int[] resids = {R.raw.sound_1, R.raw.sound_2, R.raw.sound_3, R.raw.sound_4,
                        R.raw.sound_5, R.raw.sound_6, R.raw.sound_7, R.raw.sound_8};
                LoopedSoundCollection sc = new LoopedSoundCollection(TestClient.this, resids, 4);
                id = sc.activateLeastRecent();
                sc.setVolPan(id, 1.0f, -1.0f);
                sc.play(id);
            }
        };
        thread.start();
    }
}
