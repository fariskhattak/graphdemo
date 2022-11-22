package graphdemo;

/**
 * Describes a city with a given name and unique integer identifier
 * 
 * @author William Duncan
 * 
 *         <pre>
 * Date: 99-99-9999
 * Course: csc 3102
 * Programming Project # 3 
 * Instructor: Dr. Duncan
 * Note: DO NOT MODIFY THIS FILE
 *
 * DO NOT REMOVE THIS NOTICE (GNU GPL V2):
 * Contact Information: duncan@csc.lsu.edu
 * Copyright (c) 2020 William E. Duncan
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>
 * Note: DO NOT MODIFY THIS FILE
 *         </pre>
 */
public class City implements Comparable<City> {
   /**
    * name of label of the city
    */
   private String label;
   /**
    * unique code for this city
    */
   private Integer key;

   /**
    * Creates a city with the specified code and label
    * 
    * @param aKey   unique code for the city
    * @param aLabel label for this city
    */
   public City(Integer aKey, String aLabel) {
      label = aLabel;
      key = aKey;
   }

   /**
    * Creates an anonymous city with the specified code
    * 
    * @param aKey code for the city
    */
   public City(Integer aKey) {
      label = " ";
      key = aKey;
   }

   /**
    * Gives the label associated with this city
    * 
    * @return the label of this city
    */
   public String getLabel() {
      return label;
   }

   /**
    * Gives the unique code for this city
    * 
    * @return the code for this city
    */
   public Integer getKey() {
      return key;
   }

   /**
    * Compares the code for two cities
    * 
    * @param another a reference to a city
    * @return -1 when the key for this city comes after the
    *         specified city, 0 when the keys are identical, otherwise, 1.
    */
   public int compareTo(City another) {
      return key.compareTo(another.key);
   }
}
