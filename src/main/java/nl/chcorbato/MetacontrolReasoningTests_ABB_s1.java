package nl.chcorbato;

import java.io.File;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.dlsyntax.renderer.DLSyntaxObjectRenderer;
import org.semanticweb.owlapi.formats.PrefixDocumentFormat;
import org.semanticweb.owlapi.io.OWLObjectRenderer;
import org.semanticweb.owlapi.model.AddAxiom;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLAxiom;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLClassAssertionAxiom;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLDataProperty;
import org.semanticweb.owlapi.model.OWLDataPropertyAssertionAxiom;
import org.semanticweb.owlapi.model.OWLLiteral;
import org.semanticweb.owlapi.model.OWLNamedIndividual;
import org.semanticweb.owlapi.model.OWLObjectProperty;
import org.semanticweb.owlapi.model.OWLObjectPropertyExpression;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyCreationException;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.model.OWLOntologyStorageException;
import org.semanticweb.owlapi.reasoner.OWLReasoner;
import org.semanticweb.owlapi.reasoner.OWLReasonerFactory;
import org.semanticweb.owlapi.reasoner.SimpleConfiguration;
import org.semanticweb.owlapi.util.AutoIRIMapper;

import com.clarkparsia.owlapi.explanation.DefaultExplanationGenerator;
import com.clarkparsia.owlapi.explanation.util.SilentExplanationProgressMonitor;
import com.clarkparsia.pellet.owlapiv3.PelletReasonerFactory;

import uk.ac.manchester.cs.owl.explanation.ordering.ExplanationOrderer;
import uk.ac.manchester.cs.owl.explanation.ordering.ExplanationOrdererImpl;
import uk.ac.manchester.cs.owl.explanation.ordering.ExplanationTree;
import uk.ac.manchester.cs.owl.explanation.ordering.Tree;

/**
 * @author chcorbato
 *
 */
public class MetacontrolReasoningTests_ABB_s1 {

	private static final File ONTOLOGY_APP_FILE = new File("/home/chcorbato/mros_ws/tomasys/abb_scenario1.owl");
	private static final File TOMASYS_FILE = new File("/home/chcorbato/mros_ws/tomasys/tomasys.owl");

	private static final String ONTOLOGY_APP_IRI = "http://abb_scenario1";
	private static final String TOMASYS_IRI = "http://metacontrol.org/tomasys";
	private static OWLObjectRenderer renderer = new DLSyntaxObjectRenderer();
	
	//prepare ontology and reasoner
    static OWLOntologyManager manager;
    static OWLOntology ontology;
    static OWLOntology tomasys;
    static OWLReasonerFactory reasonerFactory;
    static OWLReasoner reasoner;
    static OWLDataFactory factory;
    static PrefixDocumentFormat om;
    static PrefixDocumentFormat tm;
    
    //get tomasys classes
    static OWLClass componentState;
    static OWLClass componentClass;
    static OWLClass function;
    static OWLClass objective;
    static OWLClass functionDesign;
    static OWLClass functionGrounding;
    static OWLObjectProperty typeF;
    static OWLObjectProperty solves;
    static OWLObjectProperty fd_error_log;
    static OWLObjectProperty roles;
    static OWLObjectProperty roleDef;
    static OWLDataProperty realisability;
    static OWLDataProperty fd_efficacy;
	
	public static void main(String[] args) throws OWLOntologyCreationException {
		manager = OWLManager.createOWLOntologyManager();
		tomasys = manager.loadOntologyFromOntologyDocument(TOMASYS_FILE); // load 1st tomasys so that the manager can resolve the import
		ontology = manager.loadOntologyFromOntologyDocument(ONTOLOGY_APP_FILE);

	    reasonerFactory = PelletReasonerFactory.getInstance();
	    reasoner = reasonerFactory.createReasoner(ontology, new SimpleConfiguration());
	    factory = manager.getOWLDataFactory();
	    om = manager.getOntologyFormat(ontology).asPrefixOWLOntologyFormat();
		om.setDefaultPrefix(ONTOLOGY_APP_IRI + "#");// TODO Auto-generated constructor stub
	    tm = manager.getOntologyFormat(tomasys).asPrefixOWLOntologyFormat();
		tm.setDefaultPrefix(TOMASYS_IRI + "#");// TODO Auto-generated constructor stub

	    //get tomasys classes
	    componentState = factory.getOWLClass(":ComponentState", tm);
	    componentClass = factory.getOWLClass(":ComponentClass", tm);
	    function = factory.getOWLClass(":Function", tm);
	    objective = factory.getOWLClass(":Objective", tm);
	    functionDesign = factory.getOWLClass(":FunctionDesign", tm);
	    functionGrounding = factory.getOWLClass(":FunctionGrounding", tm);
	    typeF = factory.getOWLObjectProperty(":typeF", tm);
	    solves = factory.getOWLObjectProperty(":solves", tm);
	    fd_error_log = factory.getOWLObjectProperty(":fd_error_log", tm);
	    roles = factory.getOWLObjectProperty(":roles", tm);
	    roleDef = factory.getOWLObjectProperty(":roleDef", tm);
	    realisability = factory.getOWLDataProperty(":realisability", tm);
	    fd_efficacy = factory.getOWLDataProperty(":fd_efficacy", tm);
        
        
        /** 
         * MONITORING INPUT (from Components loop o similar) 
         * e.g. SET the status of components in the ontology (metacontrol sensory input) 
         */
        // TODO complete metacontrol perception SWRL
        ontology = updateComponentStates(manager, ontology, reasoner, factory, tm); // TODO replace with real update using ROS introspection
		
        /**
         * DIAGNOSIS REASONING
         */
	    //Ontology is updated, SWRL are triggered
	    reasoner.flush();
	    try {
			manager.saveOntology(ontology);
		} catch (OWLOntologyStorageException e) {
			e.printStackTrace();
		}
        
	    
	    /**
	     * TESTs
	     */
	    String objective = ":o_detect_ws1_tag";  // CHANGE TO TEST ANOTHER OBJECTIVE
	    
        //get values of selected properties of the individual
        OWLDataProperty o_status = factory.getOWLDataProperty(":o_status", tm);
        OWLNamedIndividual o = factory.getOWLNamedIndividual(objective, om);

        // print the status of that objective
        for (OWLLiteral ind : reasoner.getDataPropertyValues(o, o_status)) {
            System.out.println("objective " + renderer.render(o) + " status= " + ind.getLiteral());
        }
        

        /** RECONFIGURATION REASONING
         * compute the best Function Design possible to address objectives in false status 
         * TODO: do for all objectives needed
         */      
        OWLNamedIndividual fd = obtainBestFunctionDesign(o);
        System.out.println("Best FunctionDesign: " + renderer.render(fd) );
        
	    
	    /**
	     * OUTPUT components specification for RECONFIGURATION
	     */
        System.out.println("\nComponent Specifications from RECONFIGURATION REASONING:");           

        Set<OWLNamedIndividual> cspecs = new HashSet<OWLNamedIndividual>();
        for (OWLNamedIndividual rol : reasoner.getObjectPropertyValues(fd, roles).getFlattened() ) {
        	
            for (OWLNamedIndividual c : reasoner.getObjectPropertyValues(rol, roleDef).getFlattened() ) {
            	cspecs.add( c );
                System.out.println(renderer.render(c));           
            }

        }
        
	}
	
	/**
	 * Updates the status of the components in the system
	 * @param manager
	 * @param ontology
	 * @param reasoner
	 * @param factory
	 * @param pm
	 * @return
	 */
    private static OWLOntology updateComponentStates(OWLOntologyManager manager, OWLOntology ontology, OWLReasoner reasoner, OWLDataFactory factory, PrefixDocumentFormat pm) {
    	//get tomasys classes and properties
        OWLClass componentState = factory.getOWLClass(":ComponentState", pm);
        
        //get values of selected properties on the individual
        OWLDataProperty c_status = factory.getOWLDataProperty(":c_status", pm);

        OWLDataPropertyAssertionAxiom axiom;
        for (OWLNamedIndividual ind : reasoner.getInstances(componentState, false).getFlattened()) {
        	// componentState update, motor4 in error
			if (ind == factory.getOWLNamedIndividual(":motor4", pm)) 
				axiom = factory.getOWLDataPropertyAssertionAxiom(c_status, ind, false);
				
		    else
		    	axiom = factory.getOWLDataPropertyAssertionAxiom(c_status, ind, true);	

			//apply changes - add axioms
			manager.applyChange(new AddAxiom(ontology, axiom));
  		
            // print values given to component statuses - does not work for the values added here ??
            for (OWLLiteral value : reasoner.getDataPropertyValues(ind, c_status)) {
                System.out.println("Component " + renderer.render(ind) + " status= " + value.getLiteral());
            }
    		
	    }//Close for
            	
    	return ontology; 	
    
    }
    
    /**
     * REASONING best available FD for objective_ins in error
     * @param objective_ins the objective in error
     * @return besdt_fd the best FD available (fd_efficacy is the criteria)
     */
    private static  OWLNamedIndividual obtainBestFunctionDesign(OWLNamedIndividual objective_ins) {
    	
    	// obtain the typeF(unction) of the objective
        OWLNamedIndividual function_ins = reasoner.getObjectPropertyValues(objective_ins, typeF).getFlattened().iterator().next();
        
        // obtain available function designs for that function (using the inverse property)
        OWLObjectPropertyExpression inverse = factory.getOWLObjectInverseOf(solves);
       
        float comp = 0;
        OWLNamedIndividual best_fd = null;
        
        // get FDs for F
        Set<OWLNamedIndividual> fds = reasoner.getObjectPropertyValues(function_ins, inverse).getFlattened();
                
        for (OWLNamedIndividual fd : fds) {	
        	
        	for ( OWLLiteral ind : reasoner.getDataPropertyValues(fd, realisability) ) {
        		System.out.println("FD " + renderer.render(fd) + " realisability: " + ind.getLiteral());
        	}
        	// FILTER if FD realisability is NOT FALSE (TODO check SWRL are complete for this)
        	if (  !reasoner.isEntailed( factory.getOWLDataPropertyAssertionAxiom(realisability, fd, false) ) ) {    
        		
        		// FILTER if the FD error log does NOT contain the current objective
        		if ( !reasoner.getObjectPropertyValues(fd, fd_error_log).containsEntity(objective_ins) ) {
		        	for ( OWLLiteral conf : reasoner.getDataPropertyValues(fd, fd_efficacy) ) {
		        		float value = Float.parseFloat(conf.getLiteral());
		        		if (value > comp) {
		        			best_fd = fd;
		        			comp = value;
		                }//Close if
		        	}//Close for
        		}//Close if
        	}//Close if
        	
        	
        }//Close for
    	return best_fd;
    }
    
    /**
     * @author Martin Kuba makub@ics.muni.cz
     */
    private static void printIndented(Tree<OWLAxiom> node, String indent) {
        OWLAxiom axiom = node.getUserObject();
        System.out.println(indent + renderer.render(axiom));
        if (!node.isLeaf()) {
            for (Tree<OWLAxiom> child : node.getChildren()) {
                printIndented(child, indent + "    ");
            }
        }
    }

}
