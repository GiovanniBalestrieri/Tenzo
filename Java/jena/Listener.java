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
import org.apache.jena.util.iterator.ExtendedIterator;
import org.apache.jena.ontology.Individual;
import org.apache.jena.rdf.model.*;
import org.apache.jena.vocabulary.*;
import org.apache.jena.ontology.Ontology;
import org.apache.jena.ontology.OntResource;
import org.apache.jena.ontology.OntModel;
import org.apache.jena.ontology.OntClass;
import org.apache.jena.ontology.OntModelSpec;


import java.io.FileWriter;
import java.io.IOException;


import org.apache.jena.reasoner.Reasoner;
import org.apache.jena.reasoner.ReasonerRegistry;
import org.apache.jena.reasoner.ValidityReport;
import org.apache.jena.reasoner.ValidityReport.Report;


import org.apache.jena.ontology.OntDocumentManager;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

/**
 * A simple {@link Subscriber} {@link NodeMain}.
 */
public class Listener extends AbstractNodeMain {

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("rosjava/huric");
  }

  @Override
  public void onStart(ConnectedNode connectedNode) {
    final Log log = connectedNode.getLog();	
    final String JENA = "/home/userk/catkin_ws/rosjava/src/huric/my_pub_sub_tutorial/src/main/java/com/github/huric/my_pub_sub_tutorial/";
    final String SOURCE = "http://www.userk.co.uk/ontologies/ontology.owl";
    final String NS = SOURCE + "#";
    final String fileName = "ontology_modified.owl";    
    final String absoluteFileName = JENA + fileName;

	OntModel base = ModelFactory.createOntologyModel( OntModelSpec.OWL_MEM );
	OntDocumentManager dm = base.getDocumentManager();
	dm.addAltEntry(SOURCE,"file:" + JENA + "ontology.owl" );
	base.read(SOURCE,"RDF/XML");

	// Create  an inference ontology model
	OntModel inf = ModelFactory.createOntologyModel( OntModelSpec.OWL_MEM_MICRO_RULE_INF, base);
	
	// create a dummy prof for this example
	OntClass lecturerClass = base.getOntClass( NS + "Lecturer" );

	System.out.println("\n\n---- Assertions in the data ----\n\n");

	// list the asserted types

	Individual p1 = base.createIndividual( NS + "lec_ind", lecturerClass );
	for (Iterator<Resource> i = p1.listRDFTypes(false); i.hasNext(); ) {
	    System.out.println( p1.getURI() + " is asserted in class " + i.next() );
	}

	System.out.println("\n\n---- Inferred Assertions ----\n\n");

	// list the inferred types
	p1 = inf.getIndividual( NS + "lec_ind" );
	for (Iterator<Resource> i = p1.listRDFTypes(false); i.hasNext(); ) {
	    System.out.println( p1.getURI() + " IS A " + i.next() );
	}

	System.out.println("\n\n---- List all subclasses of Lecturer Class ----\n\n");

	for(Iterator<OntClass> i = lecturerClass.listSubClasses() ; i.hasNext();) {
		OntClass c = i.next();
		System.out.println(c.getURI());
	}

	System.out.println("\n\n---- List all Instances of Lecturer ----\n\n");

	lecturerClass = inf.getOntClass( NS + "Lecturer" );
	for(ExtendedIterator<? extends OntResource> i = lecturerClass.listInstances() ; i.hasNext();) {
		System.out.println(i.next());
	}

System.out.println("\n\n---- Consistency Check ----\n\n");
       
	FileWriter out = null;

	ValidityReport validity = inf.validate();
	if (validity.isValid()) {
	    System.out.println("Consistency Check:\n Passed\n");
            System.out.println("Writing to file:\n\t"+fileName);
	    try{
		out = new FileWriter(absoluteFileName);
	    	inf.write(out,"RDF/XML");
		}
	    catch (IOException a){
		System.out.println(" Occhio");
		}
		finally{
			if (out!=null){
				try {out.close();} catch(IOException ex)	{}
				} 
	}} else {
	    System.out.println("Consistency Check:\n Conflicts\n");
	    for (Iterator i = validity.getReports(); i.hasNext(); ) {
	        System.out.println(" - " + i.next());
	    }
	}


	System.out.println("\n\n---- [ The End ] ----\n\n");

   Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber("chatter", std_msgs.String._TYPE);
    subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
      @Override
      public void onNewMessage(std_msgs.String message) {
        log.info("I heard: \"" + message.getData() + "\"");
      }
    });
  }
}
