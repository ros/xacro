^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package xacro
^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.10.0 (2015-03-13)
-------------------
* security measure: forbid access to __builtins__ in expressions
* fixed evaluation issue #49:
  - literal evaluation should only consider literals, but no expressions
    use ast.literal_eval()
  - removed eval() from xacro:if evaluation
* back to string comparison to handle (lowercase) true and false
* added test cases to check
  - for equality expressions in <xacro:if>
  - math function usage
* python based evaluation of expressions
  - replaced handle_expr with python-internal eval() call
  - care has been taken to resolve variables recursively on demand 
    (in Table.__getitem__)
  - allows for evaluation of standard math functions
  - other desired functions could be added in eval_self_contained
  - Values in Table symbols are not stored as strings but as typed values.
* If text is required, a conversion with str() is performed, to ensure 
  proper evaluation of expressions. Otherwise 3*"1" would evaluate to "111".
* use __future__.division we can handle integer division evaluating to 
  floating-point devision, as before
* allow variable names for filename attribute in <xacro:include>
* allow for ordered XML processing to avoid issues with multiply defined
  properties and macros in (typically 3rd party) include files
  - enable the new behaviour by passing --inorder cmdline option
  - to improve code readibility and reusability, introduced functions
* process_include(node), grab_macro(elt, macros), grab_property(elt, symbols)
  containing 1:1 corresponding handling from process_includes, grab_macros, 
  and grab_properties
  - added corresponding test case test_inorder_processing()
* dynamic macro names using <xacro:call macro=""/>
* fixup unittests and handling of non-element nodes in <include>, <if>, <macro>
* updated pr2 gold standard to include all comments
* allow to ignore comments in nodes_match()
* New handling of non-element nodes invalidates pr2 gold standard (adding
  a lot more comments). To allow validation, allow to ignore all
  comments in comparison (as before).
* fixed handling of non-element nodes in <include>, <if>, <macro>
* fixed writexml: text nodes were not printed when other siblings exist
  - print all text, but skip whitespace-only text nodes
* improved xml matching
  - so far only element nodes (with its attributes) were considered
  - now also consider TEXT, CDATA, and COMMENT nodes
  - added function text_matches (normalizing consecutive whitespace 
    to a single space)
  added some new unit tests
  - test_consider_non_elements:
  non-element nodes are not yet considered in <if> and <macro>
* travis-ci: use catkin_make
* travis-ci: fixup running of tests
* fix pathnames used in test case
* Include CATKIN_ENV params at build time.
* use output filename flag instead of shell redirection
* create output file after parsing is complete, not before
* Contributors: Robert Haschke, Mike O'Driscoll, Morgan Quigley, William Woodall

1.9.3 (2015-01-14)
------------------
* merge test cases
* add a snapshot of the pr2 model to the test directory. add a test case which verifies that the pr2 model is parsed equal to a 'golden' parse of it.
* add more tests
* add default arg tests
* Allow default values for substitution args
* Fix up comments
* Allow xacro macros to have default parameters
* Contributors: Paul Bovbel, Morgan Quigley

1.9.2 (2014-07-11)
------------------
* add a few more tests to exercise the symbol table a bit more
* allow for recursive evaluation of properties in expressions
* add useful debugging information when parameters are not set
* stop test from failing the second time it is run
* unified if/unless handling, correctly handle floating point expressions
* floating point expressions not equal zero are now evaluated as True
* changed quotes to omit cmake warning
* Contributors: Robert Haschke, Mike Ferguson

1.9.1 (2014-06-21)
------------------
* fixup tests so they run
* export architecture_independent flag in package.xml
* installed relocatable fix
* Contributors: Michael Ferguson, Mike Purvis, Scott K Logan

1.9.0 (2014-03-28)
------------------
* Remove the roslint_python glob, use the default one.
* Add roslint target to xacro; two whitespace fixes so that it passes.
* fix evaluation of integers in if statements
  also added a unit test, fixes `#15 <https://github.com/ros/xacro/issues/15>`_
* fix setting of _xacro_py CMake var, fixes `#16 <https://github.com/ros/xacro/issues/16>`_
* Add support for globbing multiple files in a single <xacro:include>
* code cleanup and python3 support
* check for CATKIN_ENABLE_TESTING

1.8.4 (2013-08-06)
------------------
* Merge pull request `#9 <https://github.com/ros/xacro/issues/9>`_ from davetcoleman/hydro-devel
  Xacro should not use plain 'include' tags but only namespaced ones.
* Fix for the fact that minidom creates text nodes which count as child nodes
* Removed <uri> checking and made it more general for any child element of an <include> tag
* Removed Groovy reference, only being applied to Hydro
* Created check for Gazebo's <uri> tabs only only shows deprecated warnings if not present.
* Small spelling fix
* Xacro should not use plain 'include' tags but only namespaced ones.
* Merge pull request `#8 <https://github.com/ros/xacro/issues/8>`_ from piyushk/hydro-devel-conditional
  xacro conditional blocks
* using refined arguments instead of sys.argv for xml file location
* adding conditional blocks to xacro

1.8.3 (2013-04-22)
------------------
* bumped version to 1.8.3 for hydro release
* backwards compatilibity with rosbuild
* adding unit test for substitution args
* Adding supoprt for substitution_args 'arg' fields
* Remove bin copy of xacro.py
* 1.7.3
* Install xacro.py as a program so it can be run
* 1.7.2
* fixed build issues introduced in catkinization
* 1.7.1
* PEP8, cleanup, and remove roslib
* Update copyright, self import guard, and catkinize
* Catkinize.
* Cleanup in preparation of catkinization.
* Added tag unstable for changeset 169c4bf30367
* Added tag xacro-1.6.1 for changeset fc45af7fdada
* 1.6.1 marker
* xacro: fuerte compat with sub args import
* Added tag unstable for changeset 2d3c8dbfa3c9
* Added tag xacro-1.6.0 for changeset e4a4455189bf
* 1.6.0
* converted to unary stack from common stack
* xacro: fixed inserting property blocks (ros-pkg `#4561 <https://github.com/ros/xacro/issues/4561>`_)
* xacro now uses XacroExceptions. String exceptions are not allowed in Python anymore. `#4209 <https://github.com/ros/xacro/issues/4209>`_
* Added Ubuntu platform tags to manifest
* Xacro now places comments below <?xml> tag (`#3859 <https://github.com/ros/xacro/issues/3859>`_)
* Xacro prints out cleaner xml.
  Elements are now often separated by a newline.
* xacro dependency on roslaunch removed `#3451 <https://github.com/ros/xacro/issues/3451>`_
* Xacro now adds a message mentioning that the file was autogenerated (`#2775 <https://github.com/ros/xacro/issues/2775>`_)
* Remove use of deprecated rosbuild macros
* Integers stay integers in xacro, fixing `#3287 <https://github.com/ros/xacro/issues/3287>`_
* Tests for r25868
* Added a flag for only evaluating include tags in xacro
* Allowing multiple blocks and multiple insert_blocks, fixing `#3322 <https://github.com/ros/xacro/issues/3322>`_ and `#3323 <https://github.com/ros/xacro/issues/3323>`_
* doc review completed for xacro
* adding mainpage for xacro doc review
* Added xacro.cmake file that exports new xacro_add_xacro_file() macro, `#3020 <https://github.com/ros/xacro/issues/3020>`_
* Namespaced "include" tag in xacro
* Marked xacro as api reviewed
* Xacro now correctly declares the namespaces of the included documents in the final
* Made xacro accept xml namespaces
* Xacro now errors hard when a property is used without being declared
* Xacro no longer allows you to create properties with "${}" in the name
* Added the ability to escape "${" in xacro
* Made the tests in xacro run again.
* Created xacro/src
* migration part 1
