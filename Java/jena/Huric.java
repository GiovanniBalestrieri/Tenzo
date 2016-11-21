/*
 * Copyright (C) 2014 userk.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.github.rosjava.huric.my_pub_sub_tutorial;

import java.util.Iterator;
import org.apache.jena.util.FileManager;
import org.apache.jena.util.PrintUtil;
import org.apache.jena.util.iterator.ExtendedIterator;
import org.apache.jena.ontology.Individual;
import org.apache.jena.rdf.model.*;
import org.apache.jena.vocabulary.*;
import org.apache.jena.ontology.Ontology;
import org.apache.jena.ontology.OntResource;
import org.apache.jena.ontology.OntModel;
import org.apache.jena.ontology.OntProperty;
import org.apache.jena.ontology.OntClass;
import org.apache.jena.ontology.OntModelSpec;
import org.apache.jena.rdf.model.NodeIterator;
import org.apache.jena.rdf.model.RDFNode;
import java.io.FileWriter;
import java.io.IOException;

import org.apache.jena.reasoner.Reasoner;
import org.apache.jena.reasoner.ReasonerRegistry;
import org.apache.jena.reasoner.ValidityReport;
import org.apache.jena.reasoner.ValidityReport.Report;
import org.apache.jena.ontology.OntDocumentManager;
import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;


import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

/**
 * A simple {@link Publisher} {@link NodeMain}.
 */
public class Huric extends AbstractNodeMain {

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("rosjava/talker");
	}

	public void printStatements(Model m, Resource s, Property p, Resource o) {
		for (StmtIterator i = m.listStatements(s,p,o); i.hasNext(); ) {
			Statement stmt = i.nextStatement();
			System.out.println(" - " + PrintUtil.print(stmt));
		}
	}

	public void printAllProperties(Individual thisInstance){
		for (StmtIterator j = thisInstance.listProperties(); j.hasNext(); ) {
			Statement s = j.next();
			System.out.print( "\t" + s.getPredicate().getLocalName() + " -> " );
			if (s.getObject().isLiteral()) {
				System.out.println( s.getLiteral().getLexicalForm() );
			} else {
				System.out.println( s.getObject() );
			}
		}
	}


	@Override
	public void onStart(final ConnectedNode connectedNode) {
		final Log log = connectedNode.getLog();

		final String JENA_PATH = "/home/userk/catkin_ws/rosjava/src/huric/my_pub_sub_tutorial/src/main/java/com/github/huric/my_pub_sub_tutorial/";
		final String SOURCE = "http://www.semanticweb.org/ontologies/2016/1/";
		final String test_file = "ontology.owl";
		final String fileName = "a_box_mod.owl";
		final String OWL = ".owl";
		final String TBOX_FILE = "semantic_mapping_domain_model";
		final String ABOX_FILE = "semantic_map1";
		final String absoluteFileName = JENA_PATH + fileName;
		final String NS = SOURCE + TBOX_FILE + "#";
		boolean listAllProps = false;
		final String COORD_X = "float_coordinates_x";
		final String COORD_Y = "float_coordinates_y";
		final String COORD_Z = "float_coordinates_z";
		final String FURNITURE = "Furniture";
		final boolean debug = false;
		boolean isFurniture;

	    /**
	      * Tbox and Abox
	      */
	    
	    OntModel tbox = ModelFactory.createOntologyModel( OntModelSpec.OWL_MEM );
	    OntDocumentManager dm_tbox = tbox.getDocumentManager();
	    dm_tbox.addAltEntry(SOURCE+TBOX_FILE,"file:" + JENA_PATH + TBOX_FILE + OWL );
	    tbox.read(SOURCE+TBOX_FILE,"RDF/XML");


	    OntModel abox = ModelFactory.createOntologyModel( OntModelSpec.OWL_MEM);
	    OntDocumentManager dma = abox.getDocumentManager();
	    dma.addAltEntry( SOURCE + ABOX_FILE , "file:" + JENA_PATH + ABOX_FILE + OWL );
	    abox.read(SOURCE + ABOX_FILE,"RDF/XML");
	   
	    /** 
	     * Instead of creating an inference ontology model
	     * based on only the tBox, we will create a specialized 
	     * reasoner on tbox and apply inferences on abox.
	     * Thus, instead of:
	     *  OntModel inf = ModelFactory.createOntologyModel( OntModelSpec.OWL_MEM_MICRO_RULE_INF, base);
	     */
	    
	    Reasoner reasoner = ReasonerRegistry.getOWLReasoner();
	    reasoner = reasoner.bindSchema(tbox);
	    OntModelSpec ontModelSpec=OntModelSpec.OWL_MEM_MICRO_RULE_INF;
	    ontModelSpec.setReasoner(reasoner);
	    InfModel infmodel = ModelFactory.createInfModel(reasoner,abox);

	    System.out.println("\n\n-- All you know about an instance --\n\n");

	    /* Find out all we know about an instance */

	    OntProperty affordance=tbox.getOntProperty(NS+"hasAffordance");
	    OntProperty position = tbox.getOntProperty(NS+"hasPosition");

	    OntClass chair = tbox.getOntClass(NS + "Chair");

	    Resource chairRes = infmodel.getResource(NS+ "Chair");
	    System.out.println("Chair *:");
	    printStatements(abox, chairRes, null, null);

	    /* Loop through all instances */

	    System.out.println("\n\n---- List all classes of ABox ----\n\n");
	    ExtendedIterator classes = abox.listClasses(); 

	    while (classes.hasNext()) {
			// Reset furniture flag
			isFurniture = false;
	    	OntClass thisClass = (OntClass) classes.next();
			ExtendedIterator superclasses = thisClass.listSuperClasses();
		        
			if (debug)
	        	System.out.println("\nAnalyzing: " + thisClass.getURI() + " ...\n\n");

	        while (superclasses.hasNext()) {
	            OntClass c = (OntClass) superclasses.next();
	            if (!c.isAnon() && c.getURI().equals(NS+FURNITURE)){
    	    		// Toggle furniture flag
    	    		if (debug)
                		System.out.println("\t[ FURNITURE ]");
                	isFurniture = true;
                }
        	}

	    	ExtendedIterator instances = thisClass.listInstances();
	    	while (instances.hasNext()) {
	    		Individual thisInstance = (Individual) instances.next();

	    		if (thisInstance.hasProperty(position) && isFurniture) {
					System.out.println("\n\n\n -------------------- \n\n\n");
	    			float cx=-1000,cy=-1000,cz=-1000;

	    			if (listAllProps){
	    				printAllProperties(thisInstance);
	    			}

		  			// Pick just position property
	    			Statement s = thisInstance.getProperty(position);

	    			if (debug){
		    			System.out.print( "\t\t\t\tgetPredicate:\t" + s.getPredicate().getLocalName() + " -> " );
		    			System.out.println( s.getObject().toString() + "\n");
					}

	    			Individual pos = abox.getIndividual(s.getObject().toString());  
	    			for (StmtIterator j = pos.listProperties(); j.hasNext(); ) {
	    				Statement coord = j.next();
	    				if (coord.getObject().isLiteral()) {
	    					if (coord.getPredicate().getLocalName().equals(COORD_X)){
	    						cx = Float.parseFloat(coord.getLiteral().getLexicalForm());
	    					} else if (coord.getPredicate().getLocalName().equals(COORD_Y)) {
	    						cy = Float.parseFloat(coord.getLiteral().getLexicalForm());
	    					} else if (coord.getPredicate().getLocalName().equals(COORD_Z)) {
	    						cz = Float.parseFloat(coord.getLiteral().getLexicalForm());
	    					}         
	    				} 
	    			}    		    			
	    			System.out.println("\n\tFound Furniture: "+ /*+thisClass.toString()+" instance: " + */ thisInstance.toString() + "\n");
	    			System.out.format(" @  ( %f, %f , %f )\n\n",cx,cy,cz); 
	    		} else {
		    		// Loop throught other type of instances
	    		}
	    	}
	    }


	    System.out.println("\n\n---- Properties of chair ----\n\n");

	    for (ExtendedIterator<? extends OntResource> instances = chair.listInstances(); instances.hasNext(); ) {
	      	OntResource chairInstance = instances.next();
	      	System.out.println( "Chair instance: " + chairInstance.getProperty(affordance ).getString() );
	    }


	    System.out.println("\n\n---- Consistency Check ----\n\n");

	    /* Consistency Check */

	    FileWriter out = null;

		ValidityReport validity = infmodel.validate();
		if (validity.isValid()) {
			System.out.println("Consistency Check:\n Passed\n");
			System.out.println("Writing to file:\n\t"+fileName);
			try{
				out = new FileWriter(absoluteFileName);
				infmodel.write(out,"RDF/XML");
			}
			catch (IOException a){
				System.out.println(" Occhio");
			}
			finally{
				if (out!=null){
					try {out.close();} catch(IOException ex) {}
				}
			}
		} else {
			System.out.println("Consistency Check:\n Conflicts\n");
			for (Iterator i = validity.getReports(); i.hasNext(); ) {
				System.out.println(" - " + i.next());
			}
		}

		System.out.println("\n\n---- [ The End ] ----\n\n");

		final Publisher<std_msgs.String> publisher = connectedNode.newPublisher("chatter", std_msgs.String._TYPE);
	    // This CancellableLoop will be canceled automatically when the node shuts
	    // down.
		
		connectedNode.executeCancellableLoop(new CancellableLoop() {
	  	private int sequenceNumber;

	  	@Override
	  	protected void setup() {
	  		sequenceNumber = 0;
	  	}

	  	@Override
	  	protected void loop() throws InterruptedException {
	  		std_msgs.String str = publisher.newMessage();
	  		str.setData("Hello world! " + sequenceNumber);
	  		publisher.publish(str);
	  		sequenceNumber++;
	  		Thread.sleep(1000);
		  	}
		});
	}
}
