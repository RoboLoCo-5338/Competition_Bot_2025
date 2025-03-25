package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.Struct;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.Callable;

public class NetworkTablePublisher {

  public static StructArrayPublisher arrayPublisher;

  public static ArrayList<Runnable> periodics = new ArrayList<Runnable>();
  public static HashMap<String, StructPublisher> publishers = new HashMap<>();

  public static <T, V extends Struct<T>> StructPublisher getPublisherFor(String name, V struct) {

    if (publishers.containsKey(name)) {
      return publishers.get(name);
    } else {
      StructPublisher<T> publisher =
          NetworkTableInstance.getDefault().getStructTopic(name, struct).publish();
      publishers.put(name, publisher);
      return publisher;
    }
  }

  public static <T, V extends Struct<T>> void publishToNetworkTable(
      String name, Callable<T> getObject, V struct) {

    StructPublisher<T> publisher = NetworkTablePublisher.getPublisherFor(name, struct);

    periodics.add(
        () -> {
          try {
            publisher.set(getObject.call());
          } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
          }
        });
  }

  public static <T, V extends Struct<T>> void publishToNetworkTable(
      String name, T object, V struct) {

    StructPublisher<T> publisher = getPublisherFor(name, struct);

    publisher.set(object);
  }

  public static <T, V extends Struct<T>> void publishArrayToNetworkTable(
      String name, T[] objects, V struct) {

    StructArrayPublisher<T> arrayPublisher =
        NetworkTableInstance.getDefault().getStructArrayTopic(name, struct).publish();
    periodics.add(
        () -> {
          arrayPublisher.set(objects);
        });
  }

  public static void periodic() {
    for (Runnable func : periodics) {
      func.run(); // Call each function
    }
  }
}
