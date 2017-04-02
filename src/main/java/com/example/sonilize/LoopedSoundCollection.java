package com.example.sonilize;

import android.content.Context;
import android.media.AudioAttributes;
import android.media.SoundPool;

import java.util.LinkedList;
import java.util.NoSuchElementException;
import java.util.Vector;

/* an API for managing concurrent looped sounds, including playing the least recently used sound */
class LoopedSoundCollection {

    private SoundPool soundPool;
    private LinkedList<Integer> inactiveIDs;
    private Vector<LRVol> lrVolsById;
    private Vector<Boolean> isPlaying;
    private Vector<Integer> secondaryID;

    /* a container for left and right volumes */
    private class LRVol {
        private float leftVol;
        private float rightVol;

        private LRVol(float l, float r) {
            leftVol = l; rightVol = r;
        }
    }

    /* constructor */
    LoopedSoundCollection(Context context, int[] resids, int maxStreams) {
        AudioAttributes audioAttributes =
                (new AudioAttributes.Builder()
                    .setUsage(AudioAttributes.USAGE_ASSISTANCE_ACCESSIBILITY)
                    .setContentType(AudioAttributes.CONTENT_TYPE_UNKNOWN)).build();
        soundPool = (new SoundPool.Builder()
                        .setMaxStreams(maxStreams)
                        .setAudioAttributes(audioAttributes)).build();
        inactiveIDs = new LinkedList<>();
        lrVolsById = new Vector<>();
        lrVolsById.add(0, null);
        isPlaying = new Vector<>();
        isPlaying.add(0, null);
        secondaryID = new Vector<>();
        secondaryID.add(0, null);
        for (int id : resids) {
            int streamID = soundPool.load(context, id, 1);
            soundPool.setLoop(streamID, -1);
            soundPool.setVolume(streamID, 0.0f, 0.0f);
            lrVolsById.add(streamID, new LRVol(0.0f, 0.0f));
            isPlaying.add(streamID, false);
            secondaryID.add(streamID);
            inactiveIDs.add(streamID);
        }
    }

    /* "activate" the least recently active sound, making it available for playback */
    int activateLeastRecent() {
        try {
            return inactiveIDs.remove();
        } catch (NoSuchElementException e) {
            throw new NoSuchElementException("No more sounds left to play!");
        }
    }

    /* "deactivate" the least recently used sound, making it no longer available for playback;
     * stops playback if need be
     */
    void deactivate(int streamID) {
        if (inactiveIDs.contains(streamID)) {
            throw new IllegalStateException("Attempted to deactivate an inactive sound!");
        }
        inactiveIDs.add(streamID);
        soundPool.setVolume(streamID, 0.0f, 0.0f);
        isPlaying.add(streamID, false);
        soundPool.pause(streamID);
        soundPool.stop(streamID);
    }

    /* play the sound, if it is active */
    void play(int streamID) {
        if (inactiveIDs.contains(streamID)) {
            throw new IllegalStateException("Attempted to play an inactive sound!");
        }
        if (!isPlaying.get(streamID)) {
            int newID = soundPool.play(streamID, lrVolsById.get(streamID).leftVol,
                    lrVolsById.get(streamID).rightVol, 1, -1, 1.0f);
            secondaryID.add(streamID, newID);
        }
    }

    /* pause the sound */
    void pause(int streamID) {
        soundPool.pause(secondaryID.get(streamID));
        isPlaying.add(streamID, false);
    }

    /* set volume and pan of a sound */
    void setVolPan(int streamID, float vol, float pan) {
        if (inactiveIDs.contains(streamID)) {
            throw new IllegalStateException("Attempted to change volume/pan of an inactive sound!");
        }
        if (vol < 0.0f || vol > 1.0f)
            throw new IllegalArgumentException("Volume must be between 0.0 and 1.0");
        if (pan < -1.0f || pan > 1.0f)
            throw new IllegalArgumentException("Pan must be bewteen -1.0 and 1.0");
        float pan2 = pan / (1 + pan * pan) + 0.5f;
        float l = (1.0f - pan2) * vol;
        float r = pan2 * vol;
        soundPool.setVolume(secondaryID.get(streamID), l, r);
        lrVolsById.add(streamID, new LRVol(l, r));
    }

    /* destroy this sound collection and release the resources used */
    void release() {
        soundPool.autoPause();
        soundPool.release();
    }
}