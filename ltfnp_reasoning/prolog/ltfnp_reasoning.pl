/**  <module> ltfnp_reasoning

  Copyright (C) 2016 Jan Winkler
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE INSTITUTE FOR ARTIFICIAL INTELLIGENCE BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  @author Jan Winkler
  @license BSD
*/

:- module(ltfnp_reasoning,
	  [
	   ltfnp_object/1,
	   ltfnp_object_class/1,
	   ltfnp_add_object/2,
	   ltfnp_remove_object/1,
	   ltfnp_reasoner_call/3,
	   ltfnp_object_literal_atom/3
	  ]).


:- use_module(library('semweb/rdfs')).
:- use_module(library('owl_parser')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_owl')).


:-  rdf_meta
    ltfnp_object(r),
    ltfnp_object_class(r, r),
    ltfnp_add_object(r),
    ltfnp_remove_object(r),
    ltfnp_reasoner_call(r, r, r),
    ltfnp_object_literal_atom(r, r, r).


:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).


%% ltfnp_object(?Object) is nondet.
%
%  List all objects that are LTFnPObject objects.
%
% @param Object      Object identifier
%
ltfnp_object(Object) :-
    rdf_has(Object, rdf:type, A),
    rdf_reachable(A, rdfs:subClassOf, knowrob:'LTFnPObject').


%% ltfnp_object_class(?Class) is nondet.
%
%  List all classes that are decendents of LTFnPObject.
%
% @param Class      Class identifier
%
ltfnp_object_class(Class) :-
    owl_subclass_of(Class, knowrob:'LTFnPObject').


%% ltfnp_add_object(?Class, ?Object) is nondet.
%
% @param Class       The class type of the item to add
% @param Object      The added object instance
%
ltfnp_add_object(Class, Object) :-
    rdf_instance_from_class(Class, Object).


%% ltfnp_remove_object(?Object) is nondet.
%
% @param Object     The object to remove
%
ltfnp_remove_object(Object) :-
    rdf_retractall(Object, _, _).


%% ltfnp_reasoner_call(?Function, ?Parameters, ?Result) is nondet.
%
% Calls a function in the Java class 'LTFnPReasoner'. This is a shorthand for jpl_new... and jpl_call... specific to LTFnPReasoner.
%
% @param Function     The function to call
% @param Parameters   The parameters to pass to the function
% @param Result       The returned result
%
ltfnp_reasoner_call(Function, Parameters, Result) :-
    jpl_new('org.knowrob.ltfnp_reasoning.LTFnPReasoner', [], RR),
    jpl_call(RR, Function, Parameters, Result).


%% ltfnp_object_literal_atom(?Object, ?Property, ?Value) is nondet.
%
% Returns the given property (wrapped in a literal and represented as an atom) for the given object.
% @param Object       Object to return the property for
% @param Property     The property to return the literal wrapped/atom represented value for
% @param Value        The contained value
%
ltfnp_object_literal_atom(Object, Property, Value) :-
    owl_has(Object, Property, Literal),
    strip_literal_type(Literal, Atom),
    term_to_atom(Value, Atom).
