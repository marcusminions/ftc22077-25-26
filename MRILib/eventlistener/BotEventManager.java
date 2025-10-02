package MRILib.eventlistener;
//package main.java.MRILib.eventlistener;

import java.util.*;
import java.util.function.Consumer;
import MRILib.eventlistener.BotEvent.*;

public class BotEventManager {
    public enum EventType {
        OVERRIDE_DIRECTION
    }
    private static HashMap<EventType, List<Consumer<BotEvent>>> listeners = new HashMap<>();

    public static void subscribe(EventType eventType, Consumer<BotEvent> listener) {
        listeners.computeIfAbsent(eventType, k -> new ArrayList<>()).add(listener);
    }

    public static void broadcast(EventType eventType, BotEvent event) {
        List<Consumer<BotEvent>> eventListeners = listeners.get(eventType);
        if (eventListeners != null) {
            for (Consumer<BotEvent> listener : eventListeners) {
                listener.accept(event);
            }
        }
    }
}