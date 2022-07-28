/**
 * Contains static methods to serialize and deserialize to a file
 *
 * @author Joshua Au
 * @version 1.0
 * @since 6/24/2020
 */

package util;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;

public class Serialize {

    /**
     * Serializes the object to the specified file
     * @param object The object to be serialized
     * @param fileName The filename where the object should be serialized
     */
    public static void serialize(Object object, String fileName) {
        new File(fileName);

        try {
            FileOutputStream file = new FileOutputStream(fileName);
            ObjectOutputStream outputStream = new ObjectOutputStream(file);

            outputStream.writeObject(object);

            outputStream.close();
            file.close();
        } catch(IOException e){
            e.printStackTrace();
        }
    }

    /**
     * Deserializes the object from the specified file
     * @param fileName The filename where the serialized object is stored
     * @return Returns the object which is serialized in the given file
     */
    public static Object deserialize(String fileName) {
        try {
            FileInputStream file = new FileInputStream(fileName);
            ObjectInputStream inputStream = new ObjectInputStream(file);

            Object returnObject = inputStream.readObject();

            inputStream.close();
            file.close();

            return returnObject;
        } catch(IOException|ClassNotFoundException e){
            e.printStackTrace();
        }

        return null;
    }
}
