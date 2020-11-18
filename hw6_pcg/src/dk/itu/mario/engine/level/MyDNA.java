package dk.itu.mario.engine.level;

import java.util.Random;
import java.util.*;

//Make any new member variables and functions you deem necessary.
//Make new constructors if necessary
//You must implement mutate() and crossover()


public class MyDNA extends DNA
{
	
	public int numGenes = 0; //number of genes

	// Return a new DNA that differs from this one in a small way.
	// Do not change this DNA by side effect; copy it, change the copy, and return the copy.
	public MyDNA mutate ()
	{
		MyDNA copy = new MyDNA();
		//YOUR CODE GOES BELOW HERE

		Random rand = new Random();

		int index = rand.nextInt(this.getLength());
		char[] chrome = this.getChromosome().toCharArray();
		chrome[index] = (char)('a' + rand.nextInt(26));
		copy.setChromosome(String.valueOf(chrome));

		//YOUR CODE GOES ABOVE HERE
		return copy;
	}
	
	// Do not change this DNA by side effect
	public ArrayList<MyDNA> crossover (MyDNA mate)
	{
		ArrayList<MyDNA> offspring = new ArrayList<MyDNA>();
		//YOUR CODE GOES BELOW HERE

		Random rand = new Random();

		String thisChrome = this.getChromosome();
		String mateChrome = this.getChromosome();

		int breakpoint = rand.nextInt(thisChrome.length());

		MyDNA child1 = new MyDNA();
		MyDNA child2 = new MyDNA();

		child1.setChromosome(thisChrome.substring(0, breakpoint) + mateChrome.substring(breakpoint));
		child2.setChromosome(mateChrome.substring(0, breakpoint) + thisChrome.substring(breakpoint));

		offspring.add(child1);
		offspring.add(child2);

		//YOUR CODE GOES ABOVE HERE
		return offspring;
	}
	
	// Optional, modify this function if you use a means of calculating fitness other than using the fitness member variable.
	// Return 0 if this object has the same fitness as other.
	// Return -1 if this object has lower fitness than other.
	// Return +1 if this objet has greater fitness than other.
	public int compareTo(MyDNA other)
	{
		int result = super.compareTo(other);
		//YOUR CODE GOES BELOW HERE
		
		//YOUR CODE GOES ABOVE HERE
		return result;
	}
	
	
	// For debugging purposes (optional)
	public String toString ()
	{
		String s = super.toString();
		//YOUR CODE GOES BELOW HERE
		
		//YOUR CODE GOES ABOVE HERE
		return s;
	}
	
	public void setNumGenes (int n)
	{
		this.numGenes = n;
	}

}

