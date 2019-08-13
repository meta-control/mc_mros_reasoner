package nl.chcorbato;

import java.io.File;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.change.CreateValuePartition;
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
import org.semanticweb.owlapi.util.DefaultPrefixManager;

import com.clarkparsia.owlapi.explanation.DefaultExplanationGenerator;
import com.clarkparsia.owlapi.explanation.util.SilentExplanationProgressMonitor;
import com.clarkparsia.pellet.owlapiv3.PelletReasoner;
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

	private static final File ONTOLOGY_APP1_FILE = new File("/home/chcorbato/mros_ws/tomasys/abb_scenario1.owl");
	private static final File ONTOLOGY_APP2_FILE = new File("/home/chcorbato/mros_ws/tomasys/abb_scenario2.owl");

	private static final File TOMASYS_FILE = new File("/home/chcorbato/mros_ws/tomasys/tomasys.owl");

	private static final String ONTOLOGY_APP1_IRI = "http://abb_scenario1";
	private static final String ONTOLOGY_APP2_IRI = "http://abb_scenario2";

	private static final String TOMASYS_IRI = "http://metacontrol.org/tomasys";
	private static OWLObjectRenderer renderer = new DLSyntaxObjectRenderer();
	
	//prepare ontology and reasoner
    static OWLOntologyManager manager;
    static OWLOntology ontology;
    static OWLOntology tomasys;
    static OWLReasonerFactory reasonerFactory;
    static PelletReasoner reasoner;
    static OWLDataFactory factory;
    static PrefixDocumentFormat om;
    static PrefixDocumentFormat tm;
    
    // tomasys classes
    static OWLClass componentState;
    static OWLClass componentClass;
    static OWLClass binding;
    static OWLClass function;
    static OWLClass objective;
    static OWLClass functionDesign;
    static OWLClass functionGrounding;
    static OWLObjectProperty typeF;
    static OWLObjectProperty solves;
    static OWLObjectProperty realises;
    static OWLObjectProperty requires;
    static OWLObjectProperty needs;
    static OWLObjectProperty fd_error_log;
    static OWLObjectProperty roles;
    static OWLObjectProperty roleDef;
    static OWLObjectProperty binding_role;
    static OWLObjectProperty binding_component;
    static OWLObjectProperty hasBindings;
    static OWLDataProperty fd_realisability;
    static OWLDataProperty fd_efficacy;
    static OWLDataProperty o_status;
    static OWLDataProperty o_performance;
    static OWLDataProperty b_status;
    static OWLDataProperty c_status;
    static OWLDataProperty c_performance;
    static OWLDataProperty cc_availability;
    static OWLDataProperty cc_unique;
    static OWLDataProperty fg_performance;
    static OWLDataProperty fg_status;

	
	public static void main(String[] args) throws OWLOntologyCreationException {
	    
		/**
		 * TEST to run: 
		 * 0..2 scenario1
		 * 3..  scenario2
		 */
	    int test = -1;
	    
	    // parse arguments [1..n]
	    try {
		    test = Integer.parseInt(args[0]);	
		} catch (Exception e) {
			System.out.println("No test specified");
		}
		
		// Load ontologies required (app + tomasys)
		manager = OWLManager.createOWLOntologyManager();
		tomasys = manager.loadOntologyFromOntologyDocument(TOMASYS_FILE); // load 1st tomasys so that the manager can resolve the import
		
		if( test < 3 ) // depending on the arg load the app ontology for scenario1 or scenario2
			ontology = manager.loadOntologyFromOntologyDocument(ONTOLOGY_APP1_FILE);
		else
			ontology = manager.loadOntologyFromOntologyDocument(ONTOLOGY_APP2_FILE);

		// create Pellet reasoner non buffered, so that modifications will be immediately visible in the reasoning results
	    reasoner = PelletReasonerFactory.getInstance().createNonBufferingReasoner(ontology, new SimpleConfiguration());
	    manager.addOntologyChangeListener( reasoner );
	    
	    factory = manager.getOWLDataFactory();
	    om = manager.getOntologyFormat(ontology).asPrefixOWLOntologyFormat();
	    if( test < 3 )
	    	om.setDefaultPrefix(ONTOLOGY_APP1_IRI + "#");
	    else
	    	om.setDefaultPrefix(ONTOLOGY_APP2_IRI + "#");

	    tm = manager.getOntologyFormat(tomasys).asPrefixOWLOntologyFormat();
		tm.setDefaultPrefix(TOMASYS_IRI + "#");

	    //get tomasys classes
	    componentState	  = factory.getOWLClass(":ComponentState",tm);         
	    componentClass    = factory.getOWLClass(":ComponentClass",tm);
	    binding    = factory.getOWLClass(":Binding",tm);         
	    function          = factory.getOWLClass(":Function",tm);         
	    objective         = factory.getOWLClass(":Objective",tm);         
	    functionDesign    = factory.getOWLClass(":FunctionDesign",tm);         
	    functionGrounding = factory.getOWLClass(":FunctionGrounding",tm);
	    typeF             = factory.getOWLObjectProperty(":typeF",tm);
	    solves            = factory.getOWLObjectProperty(":solves",tm);
	    realises            = factory.getOWLObjectProperty(":realises",tm);
	    requires		  = factory.getOWLObjectProperty(":requires",tm);
	    needs			  = factory.getOWLObjectProperty(":needs",tm);
	    fd_error_log      = factory.getOWLObjectProperty(":fd_error_log",tm);
	    roles             = factory.getOWLObjectProperty(":roles",tm);
	    roleDef           = factory.getOWLObjectProperty(":roleDef",tm);  
	    binding_role      = factory.getOWLObjectProperty(":binding_role",tm); 
	    binding_component = factory.getOWLObjectProperty(":binding_component",tm); 
	    hasBindings 	  = factory.getOWLObjectProperty(":hasBindings",tm); 
	    fd_realisability  = factory.getOWLDataProperty(":fd_realisability",tm);  
	    fd_efficacy       = factory.getOWLDataProperty(":fd_efficacy",tm);  
	    o_status          = factory.getOWLDataProperty(":o_status",tm);  
	    o_performance     = factory.getOWLDataProperty(":o_performance",tm);  
	    b_status          = factory.getOWLDataProperty(":b_status",tm);  
	    c_status          = factory.getOWLDataProperty(":c_status",tm);  
	    c_performance     = factory.getOWLDataProperty(":c_performance",tm);  
	    cc_availability   = factory.getOWLDataProperty(":cc_availability",tm);  
	    cc_unique         = factory.getOWLDataProperty(":cc_unique",tm);  
	    fg_performance    = factory.getOWLDataProperty(":fg_performance",tm);  
	    fg_status         = factory.getOWLDataProperty(":fg_status",tm);  	                   

	    /**
	     * TEST to run
	     */   
	    switch ( test ) {
	    
		    case 0: 
		    	System.out.println("\nSimple test: print objective status after injecting the fact that it is 'true'");
		    	SimpleTest();
		    	break;
		    	
		    case 1: // scenario 1 minimal reasoning = objective observed in error
		    	System.out.println("\nscenario 1 minimal reasoning = objective observed in error");
		    	String str_objective_tag = ":o_detect_ws1_tag";  // CHANGE TO TEST ANOTHER OBJECTIVE
			    //get values of selected properties of the individual
			    OWLNamedIndividual o_tag = factory.getOWLNamedIndividual(str_objective_tag, om);
		        manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(o_status, o_tag, false));
		    	break;
		    	
		    case 2: // scenario 1.b = camera in permanent error
		    	System.out.println("\nscenario 1.b = camera in permanent error");
		    	String str_camera = ":c_camera";
		    	String str_c_camera = ":cc_camera";
			    OWLNamedIndividual camera = factory.getOWLNamedIndividual(str_camera, om);
			    // camera in error
			    manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(c_status, camera, false));
			    // camera unavailable
			    reasoner.flush();
		    	break;
		    	
		    case 3: // scenario 2 = camera in permanent error
		    	System.out.println("\nscenario 2.a = Yumi config 2arms in ERROR");
		    	String str_yumi = ":c_yumi";
			    OWLNamedIndividual yumi = factory.getOWLNamedIndividual(str_yumi, om);
			    // yumi in error
			    manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(c_status, yumi, false));
			    // yumi 2arms unavailable
			    reasoner.flush();
		    	break;
		    
		    default:
				System.out.println("\nRunning default metacontrol reasoning using monitoring input");

		    	/** 
		         * MONITORING INPUT (from Components loop o similar) 
		         * e.g. SET the status of components in the ontology (metacontrol sensory input) 
		         */
		        // TODO complete metacontrol perception SWRL
		        ontology = updateComponentStates(manager, ontology, reasoner, factory, tm); // TODO replace with real update using ROS introspection, including objective observers
		    }
	    	
        /**
         * DIAGNOSIS REASONING
         */
	    //KB is updated, SWRL are triggered
	    reasoner.flush();
	    
	    // PRINT system status
	    
	    // Print Components
	    System.out.println("\nComponents:");
        Set<OWLNamedIndividual> individuals = reasoner.getInstances(componentState, false).getFlattened();
        for ( OWLNamedIndividual i : individuals ) {
        	System.out.println(renderer.render(i) + reasoner.getDataPropertyValues(i, c_status) );
        }   
        
	    // Print Bindings
        System.out.println("\nBindings:");
        individuals = reasoner.getInstances(binding, false).getFlattened();
        for ( OWLNamedIndividual i : individuals ) {
        	System.out.println(renderer.render(i) + reasoner.getDataPropertyValues(i, b_status) );
        }   
                
	    // Print FGs
	    System.out.println("\nFGs:");
        individuals = reasoner.getInstances(functionGrounding, false).getFlattened();
        for ( OWLNamedIndividual i : individuals ) {
        	System.out.println(renderer.render(i) + reasoner.getDataPropertyValues(i, fg_status) );
        }        
        
	    // Print Objectives
	    System.out.println("\nObjectives:");
        Set<OWLNamedIndividual> current_objectives = reasoner.getInstances(objective, false).getFlattened();
        for ( OWLNamedIndividual i : current_objectives ) {
        	System.out.println(renderer.render(i) + reasoner.getDataPropertyValues(i, o_status) );
        }

	    // Print ComponentClasses
	    System.out.println("\nComponent Classes:");
        individuals = reasoner.getInstances(componentClass, false).getFlattened();
        for ( OWLNamedIndividual i : individuals ) {
        	System.out.println(renderer.render(i) + reasoner.getDataPropertyValues(i, cc_availability) );
        }
        
	    // Print FDs
        individuals = reasoner.getInstances(functionDesign, false).getFlattened();
	    System.out.println("\nFDs:");
        for ( OWLNamedIndividual i : individuals ) {
        	System.out.println(renderer.render(i) + reasoner.getDataPropertyValues(i, fd_realisability) );
        }

	    

        /** RECONFIGURATION REASONING
         * compute the best Function Design possible to address objectives in false status (aka in ERROR)
         * TODO: filter out objectives no longer needed in the hierarchy
         */
	    
        // init objectives in error
        Set<OWLNamedIndividual> objectives_in_error = new HashSet<OWLNamedIndividual>();

        for ( OWLNamedIndividual i : current_objectives ) {
        	if ( !reasoner.getDataPropertyValues(i, o_status).iterator().next().parseBoolean() );
        	objectives_in_error.add(i);
        }
        
        // Ground a solution hierarchy for each root objective in error. We assume here that root_objectives do not share intermediate objectives
        Set<OWLNamedIndividual> cspecs = new HashSet<OWLNamedIndividual>();
        for ( OWLNamedIndividual o : objectives_in_error ) {	        		
        	groundObjective(o, cspecs);      
		}
	    
	    /**
	     * OUTPUT components specification for RECONFIGURATION
	     * TODO: address potential issue of overlapping cs in the fds
	     */
        System.out.println("\n== Component Specifications from RECONFIGURATION REASONING:==");           
        for ( OWLNamedIndividual cs : cspecs ) {
        	System.out.println(renderer.render(cs) );
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
        	// UNEXMIN ROBOT19 paper : componentState update, motor4 in error
			if (ind == factory.getOWLNamedIndividual(":motor4", pm)) 
				axiom = factory.getOWLDataPropertyAssertionAxiom(c_status, ind, false);
				
		    else
		    	axiom = factory.getOWLDataPropertyAssertionAxiom(c_status, ind, true);	

			//apply changes - add axioms
			manager.applyChange(new AddAxiom(ontology, axiom));
			reasoner.flush();
            // print values given to component statuses - does not work for the values added here ??
            for (OWLLiteral value : reasoner.getDataPropertyValues(ind, c_status)) {
                System.out.println("Component " + renderer.render(ind) + " status= " + value.getLiteral());
            }
    		
	    }//Close for
            	
    	return ontology; 	
    
    }
  
    
    /**
     * REASONING best available FD for objective_ins in error
     * @param o the objective in error
     * @return besdt_fd the best FD available (fd_efficacy is the criteria)
     */
    private static OWLNamedIndividual obtainBestFunctionDesign(OWLNamedIndividual o) {
    	// obtain the typeF(unction) of the objective
        OWLNamedIndividual f = reasoner.getObjectPropertyValues(o, typeF).getFlattened().iterator().next();
        
        // obtain available function designs for that function (using the inverse property)
        OWLObjectPropertyExpression inverse = factory.getOWLObjectInverseOf(solves);
       
        float comp = 0;
        OWLNamedIndividual best_fd = null;
        
        // get FDs for F
        Set<OWLNamedIndividual> fds = reasoner.getObjectPropertyValues(f, inverse).getFlattened();
                
        for (OWLNamedIndividual fd : fds) {	
        	
        	for ( OWLLiteral ind : reasoner.getDataPropertyValues(fd, fd_realisability) ) {
        		System.out.println("FD " + renderer.render(fd) + " realisability: " + ind.getLiteral());
        	}
        	// FILTER if FD realisability is NOT FALSE (TODO check SWRL rules are complete for this)
        	if (  !reasoner.isEntailed( factory.getOWLDataPropertyAssertionAxiom(fd_realisability, fd, false) ) ) {    
        		
        		// FILTER if the FD error log does NOT contain the current objective
        		if ( !reasoner.getObjectPropertyValues(fd, fd_error_log).containsEntity(o) ) {
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
    	System.out.println("\n\t~FunctionDesign: " + renderer.render(best_fd) + "\t resoned best for Objective: " + renderer.render(o));

    	return best_fd;
    }
   
    
    /**
     * To solve the given Objective, recursively Grounds the required hierarchy of
     * sub-Objectives and Function Groundings
     */
    private static Set<OWLNamedIndividual> groundObjective(OWLNamedIndividual o, Set<OWLNamedIndividual> cspecs) {
    	System.out.println("\n=GROUNDING Objective: " + renderer.render(o) );
        // obtain FD for Objective
        OWLNamedIndividual fd = obtainBestFunctionDesign(o);
        if ( fd == null ) { //TODO decide where and how to report this
	        System.out.println("\nOPERATOR ATTENTION NEEDED: there is no FunctionDesign realisable for objective: " + renderer.render(o) );
        	return cspecs;
        }
        
    	//ground fd into fg
    	OWLNamedIndividual fg = createIndividualFromClass("fg_" + renderer.render(fd) , functionGrounding);
    	manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(realises, fg, o));
      
        for (OWLNamedIndividual rol : reasoner.getObjectPropertyValues(fd, roles).getFlattened() ) {  
        	OWLNamedIndividual b = createIndividualFromClass("b_" + renderer.render(fd) , binding);	
        	manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(binding_role, b, rol));
        	manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(hasBindings, fg, b));
        	// get comp specs to deploy
        	OWLNamedIndividual cs = reasoner.getObjectPropertyValues(rol, roleDef).getFlattened().iterator().next();
        	cspecs.add( cs );
        	           
        }
               
        // recursively call grounding for each required function
        for (OWLNamedIndividual f : reasoner.getObjectPropertyValues(fd, requires).getFlattened() ) {
	    	// instantiate objective from required function
        	OWLNamedIndividual ob = createIndividualFromClass("o_" + renderer.render(f) , objective);
	    	manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(typeF, ob, f));
	    	manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(needs, fg, ob));
	    	groundObjective(ob, cspecs); // recursivity!!       	
        }
        
        return cspecs;
    }
    
	/**
	 * Simple Test
	 */
    private static void SimpleTest(){
	    String str_objective_tag = ":o_detect_ws1_tag";  // CHANGE TO TEST ANOTHER OBJECTIVE
	    
	    //get values of selected properties of the individual
	    OWLNamedIndividual o_tag = factory.getOWLNamedIndividual(str_objective_tag, om);
	    manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(o_status, o_tag, true));
	    reasoner.flush();

	    // print the status of that objective
	    for (OWLLiteral ind : reasoner.getDataPropertyValues(o_tag, o_status)) {
	        System.out.println("objective " + renderer.render(o_tag) + " status= " + ind.getLiteral());
	    }
    }
    
    
    
    /**
     * Based on:
     * CreateOntologyInCodeExample.java by Martin Kuba makub@ics.muni.cz
     */
    private static OWLNamedIndividual createIndividualFromClass(String name, OWLClass oclass) {
        OWLNamedIndividual individual = factory.getOWLNamedIndividual(name, om);
        manager.addAxiom(ontology, factory.getOWLDeclarationAxiom(individual));
        manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(oclass, individual));
        return individual;
    }

    

}
