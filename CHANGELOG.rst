^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package xacro
^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.14.5 (2020-10-13)
-------------------
* [fix]     yaml loading: recursively wrap lists and dicts for dotted dict access (`#258 <https://github.com/ros/xacro/issues/258>`_)
* [feature] Provide support for yaml constructors !degrees and !radians (`#252 <https://github.com/ros/xacro/issues/252>`_)
* Contributors: Robert Haschke, G.A. vd. Hoorn

1.14.4 (2020-08-09)
-------------------
* [fix] Rework YamlDictWrapper to restore dict properties (`#250 <https://github.com/ros/xacro/issues/250>`_)
* [fix] Ignore underscores when parsing literal numeric values (`#247 <https://github.com/ros/xacro/issues/247>`_)
* Contributors: Robert Haschke

1.14.3 (2020-07-05)
-------------------
* [feature] Improve warnings
  - Unify meaning of verbosity > 0 (to print file location)
  - Provide file location on warning in check_attrs()
  - Issue warning on child elements of <xacro:include> tag
* [feature] Allow dotted access to yaml-loaded dicts: d.key1.key2.key3 (`#245 <https://github.com/ros/xacro/issues/245>`_)
* [maint]   Travis: Update distro to Bionic
* Contributors: Robert Haschke, G.A. vd. Hoorn

1.14.2 (2020-05-21)
-------------------
* [maintanence] Remove deprecated xacro.py (`#239 <https://github.com/ros/xacro/issues/239>`_)
* Contributors: Shane Loretz

1.14.1 (2020-03-29)
-------------------
* [feature]     allow optional xacro includes (`#234 <https://github.com/ros/xacro/issues/234>`_)
* [maintanence] Use setuptools instead of distutils (`#233 <https://github.com/ros/xacro/issues/233>`_)
* [maintanence] fix Travis: export correct ROS_PYTHON_VERSION
* Contributors: Alejandro Hernández Cordero, Robert Haschke

1.14.0 (2019-12-08)
-------------------
* [maintanence] Remove deprecations
  - Require all xacro commands to be prefixed with 'xacro:'
  - Remove options --legacy, --inorder, --check-order, --includes
* Contributors: Robert Haschke

1.13.5 (2019-12-08)
-------------------
* [feature] Expose abs_filename() (`#220 <https://github.com/ros/xacro/issues/220>`_)
* [feature] Catch missing closing brace in $() and ${} expressions
* [maintanence]
  - Replace deprecated yaml.load() -> yaml.safe_load()
  - Save macro names internally w/o 'xacro:' prefix
  - Correctly issue deprecation warning for non-prefixed xacro tags
* Contributors: Robert Haschke

1.13.4 (2019-09-26)
-------------------
* [feature] remove xmlns:xacro from processed file (`#207 <https://github.com/ros/xacro/issues/207>`_)
  - Remove all notions of xmlns:xacro from the resulting document.
  - If the root node defines a xacro:targetNamespace attribute, this will become the global xmlns namespace of the resulting document.
* [feature] Add len() to allowed python functions (`#208 <https://github.com/ros/xacro/issues/208>`_)
* [maintanence]
  - --in-order warning: reduce severity level to message
  - fix and cleanup test of cmake extensions
  - adapt run_xacro() to run xacro from PATH
  - simplify import of substition_args
  - remove 'requires' field from setup.py
  - fix Travis config: use new repository key, use xenial/kinetic distro
  - basic README.md
  - fix catkin_lint issue
  - remove duplicate catkin_python_setup() (`#206 <https://github.com/ros/xacro/issues/206>`_)
* Contributors: Robert Haschke, James Xu, Martin Pecka

1.13.3 (2018-10-14)
-------------------
* use INORDER as default in cmake functions
* remove weird 'None's in error messages
* Contributors: Robert Haschke

1.13.2 (2018-05-14)
-------------------
* deprecate --includes option (which is tied to deprecated --legacy processing)
* moved all option handling to cli.py, including deprecation warnings for options
* Contributors: Robert Haschke

1.13.1 (2018-05-03)
-------------------
* fix parsing of quoted strings in default args for xacro params (`#187 <https://github.com/ros/xacro/issues/187>`_)
* Contributors: Robert Haschke

1.13.0 (2018-03-31)
-------------------
* make --inorder processing the default
* Contributors: Robert Haschke

1.12.1 (2018-03-28)
-------------------
* `#183 <https://github.com/ros/xacro/issues/183>`_: unicode support for python2 and python3
* `#178 <https://github.com/ros/xacro/issues/178>`_: extend list of allowed python builtins: min, max, round
* `#182 <https://github.com/ros/xacro/issues/182>`_: suppress xacro warnings when determining dependencies
* `#151 <https://github.com/ros/xacro/issues/151>`_: fixes for `#149 <https://github.com/ros/xacro/issues/149>`_ and `#148 <https://github.com/ros/xacro/issues/148>`_
* `#157 <https://github.com/ros/xacro/issues/157>`_: fix `#156 <https://github.com/ros/xacro/issues/156>`_ access to undefined target_table
* `#150 <https://github.com/ros/xacro/issues/150>`_: allow True/False literals in python expressions
* `#159 <https://github.com/ros/xacro/issues/159>`_: load ROS-related packages on demand, thus becoming more independent from ROS
* `#173 <https://github.com/ros/xacro/issues/173>`_: allow default values for properties
* `#172 <https://github.com/ros/xacro/issues/172>`_: fix formatting of XacroException
* `#171 <https://github.com/ros/xacro/issues/171>`_: fix dependency handling (--deps option)
* `#163 <https://github.com/ros/xacro/issues/163>`_: full python 3 compatibility
* Contributors: Robert Haschke, Kartik Mohta, Morgan Quigley, Steven Peters

1.12.0 (2017-03-25)
-------------------

1.11.2 (2017-02-27)
-------------------
* Convert exception to string in a python2/3 compatible way.
* Use python2/3 independent check for file type.
* Contributors: Hans Gaiser, Maarten de Vries

1.11.1 (2016-06-22)
-------------------
* workaround for xml.dom.minidom issue
* ensure non-empty error string
* Contributors: Robert Haschke

1.11.0 (2016-03-25)
-------------------
* added short option -i as alternative to --inorder
* refactored main to fix #122, #107
* added xacro indicator to error message to fix #123
* moved banner generation to process_file()
* removed special (but obsolete) output handling for just_includes mode
* moved core processing pipeline into function process_file()
* improved documentation: more comments, input_file -> input_file_name
* fix #120: handle non-space whitespace characters in params string
* extended tests to handle non-space whitespace characters in params string
* always store macros with xacro: prefix in front: #118
* fix #115: enforce xacro namespace usage with --xacro-ns option
* apply correct checking for include tags, and extend testcase
* allow (one-level) nested expression/extension evaluation
* Contributors: Robert Haschke, Morgan Quigley

1.10.6 (2015-09-01)
-------------------
* use correct catkin environment for cmake dependency checking
* fixed dependency definition for cmake usage
* Contributors: Robert Haschke

1.10.5 (2015-08-12)
-------------------
* fix #108: evaluate property blocks recursively too
* improved macro parameter parsing
* use a regular expression to parse a param spec with forwarding and default
* allow for spaces in default string (within single quotes)
* forwarding macro arguments from outer scope
* switched to `^|` syntax
* use more compact `$|` syntax
* moved parsing of argument defaults to grab_macro()
* explicit forwarding of properties to macro scope
* replace silent/implicit forwarding of properties from outer scope to
  macro scope by an explicit "call" to a `forward(<name>[,<default>])` function.
* implicit forwarding of outer-scope properties to macro args (#100)
* property evaluation fixes
* suppress double evaluation of properties
* adapted unittest to cover the fixed issue
* fixed evaluation order for properties exported to parent or global scope
* Merge pull request #103 from ubi-agni/overwrite-check
  issue warning when attempting to overwrite existing global property
* fixed unittest: avoid overwrite warning
* warn when overwriting any globally defined variable
* Merge pull request #102 from ubi-agni/completion
  bash completion
* Merge pull request #99 from ubi-agni/jade-devel
  reworked macro resolution
* moved test_macro_name_with_colon() to class TestXacro
  should be tested both, in oldorder and inorder mode
* bash completion
* improved error message for failed $(find)
* reworked macro resolution
  python-eval-based macro resolution (introduced to enable namespaces)
  heavily restricted the set of possible macro names (only valid python
  identifiers were allowed)
  Particularly, xacro: prefixed macro names were forbidden.
* initial attempt to fix #97
* add failing test case with colon in the macro name
* Contributors: Robert Haschke

1.10.4 (2015-06-18)
-------------------
* removed test_DEPRECATED_should_replace_before_macroexpand()
  duplicates test_should_replace_before_macroexpand()
* fixed evaluation order of macro arguments and body
  Macro arguments need to be evaluated and assigned to properties before
  body is evaluated. Otherwise, the evaluated value will be converted to
  str, i.e. loosing original type.
* Contributors: Robert Haschke

1.10.3 (2015-06-16)
-------------------
* deprecate --oldorder processing
* added --check-order option to do a simple check for --inorder compatibility
  - Most probable incompatibility is redefining a property after its usage.
  - tested and reported with file location of (first) redefinition after usage.
* moved command line processing to cli.py
* explain verbosity levels in usage string
* colorize errors during cli parsing
* log definition and usage of properties
* replaced debug option by verbosity options -q, -v
* fixed evaluation time of default macro params
* introduced Macro object to increase code readability
* parse a macro's parameter list once at declaration time (instead of every instantiation)
* extended test_multiple_blocks() to check for both normal and reversed order
* added unittest to increase code coverage
* do not issue deprecation warning for <include> tags that are non-xacro
* added option --oldorder
* allow to store properties to parent or global scope
* added <xacro:attribute>
* cleaned up error message about missing files
* moved xml-specific functions to xmlutils.py
  new generic functions opt_attrs(), reqd_attrs(), and check_attrs()
  to fetch optional and required attributes and warn about unknown ones
  in a uniform fashion
* unittest to allow empty <arg> defaults
* Merge pull request #94 from ubi-agni/minor-fixes
* minor fixes
* remove duplicates in --deps output
* fixed dependency checking for --inorder mode (which requires full processing)
* fixed doc of xacro' cmake macros
* renamed "xacro:rename" to "xacro:element"
* allow namespacing for xacro:include's
* allow renaming of element names using xacro:rename
* unittest cleanup
* check property and macro names to be valid python identifiers
* allow namespacing of xacro:include's
* properties and macros in an included file will go into their own,
  separate namespace, if the XML attribute `ns` is provided.
  Access is by standard python syntax: namespace.name
* allow renaming of element names
  <xacro:rename xacro:name="<new element name>"/>
* moved unittests requiring --inorder processing to class TestXacroInorder
* added unittest test_dynamic_macro_undefined()
* improved error message when variable include filename is used
  without --inorder
* stripped down unittest test_inorder_processing()
* improved processing
* adapted pr2 gold standard removing most comments again
  this partially reverts 59605fb1521583dc63efdea13f4c45128499bd20
* remove all XML comments directly before xacro elements
  (These are considered xacro-related only and should be removed in the final doc.)
  Leaving an empty line between xacro-unrelated and xacro-related comments
  allows to include the former.
* unittest: test_ignore_xacro_comments()
* improved processing
  - recursive (instead of iterative) eval_all()
  - reusable process_include()
  - replace_node() function to replace xacro tag by some other content
  - avoid reprocessing of nodes
  - avoid deep copy where possible (speedup)
* fix evaluation (#83)
* yaml support
* check for consistency of xml namespaces on xacro:include
* replaced strip()=='' by more efficient isspace()
* allow transitive definition of substition args
* fixed evaluation of literals in property definitions
  - literals with preceding whitespace will be silently stripped (#83)
  - more complex evaluation test (perturbing spaces added)
* fixed xacro namespaces in pr2 files to get rid of new inconsistency warning
* warning message on inconsistent namespace redefinition for includes
* yaml support
  ${load_yaml('file.yaml')} to load dict from yaml file
* Merge pull request #85 from ubi-agni/error-reporting
  improved error reporting
* nicer formatting of multiple "when evaluating expression" lines
* improved formatting of error messages
  use XacroException to wrap and augment other exceptions
  to achieve a clearer error formatting
* better error message for missing substitution args
* use colorized warnings where possible
* included macro stack in error-reporting
* maintain filestack to facilitate error reporting at any time
* Merge pull request #82 from ubi-agni/unittests
  improved unittesting
* allow to capture (and check) stderr in unit tests
* improved unittests to test both, classic and in-order processing
* Merge pull request #81 from ubi-agni/jade-devel
  Thank you for your time and contributions. Improving cosmetics is important.
* PEP8 cleanup
* cmake: only copy files to devel space if new
* Merge pull request #80 from ubi-agni/jade-devel
* improved error-handling opening the output file
  - running multiple xacro process in parallel, all writing into a new dir
  could cause a race condition when creating the dir
  - improved error message on output creation failure
* removed rospy dependency
  - Importing rospy caused build order issues with ros_comm in workspace
* Filtering out REMAP command-line arguments is done manually now.
* update authors/maintainers and copyright statements
* deprecate non-namespaced xacro tags
* added missing print_location_msg() for file that actually failed parsing
* improved deprecation warnings
* New cli option `--xacro-ns` allows to enforce the new policy
  requiring the xacro namespace prefix (and suppressing deprecation warnings).
  However, non-prefixed tags will not be modified by xacro anymore
  (as requested by #41, #59, #60).
  Partially reverted cb73cfd8c678adfda2172accef398189ea2338a1, handling
  <arg> tags in the same fashion as other tags, i.e. issue a warning if
  used without prefix and ignoring it with cli argument `--xacro-ns`.
* fixed pr2 xacro files to use 'xacro:' prefixed tags only
* fixed unittests in test_xacro.py to use 'xacro:' prefixed tags only
* deprecation message for missing xacro namespace prefix in xml tags
* moved colored warning messages into color.py (for reuseability)
* added missing print_location_msg() for file that actually failed parsing
* improved xacro's cmake macros
* prepend ${PACKAGE_NAME} to all generated cmake targets
  Otherwise multiple packages employing xacro's cmake macros will use the
  same conflicting target name.
  This is only an issue with catkin_make, which defines a single global
  cmake namespace. The new catkin tools (or catkin_make_isolated) build
  each package separately.
* basic unittest for xacro's cmake macros
* improved xacro's cmake macros
  - xacro_add_xacro_file() automatically determines output file from input (removing .xacro suffix).
  If that fails, a fatal error is raised.
  - added xacro_install() to allow installation into both, devel and install space.
  - replaced conveniency function xacro_add_files()
* Contributors: Robert Haschke

1.10.2 (2015-05-23)
-------------------

* added --debug option to explicitly enable stack traces
  By default, only show error message to the user.
  Stack traces are only interesting for xacro developers.
* recursive include processing
  - more informed error messages (which file was included from where)
  - allows relative path names for include filename specs
  they are interpreted relative to the current file
* new substitution command $(cwd) to extract current working directory
* added unittest cases
  - creation of required subdirs for output
  - recursive xacro:include
  - extended test_include_glob() to check for all glob patterns
* added run_xacro() function to simplify unittests running xacro script
* moved xacro.py back to original location
* nicely colored deprecation warning
* create required dirs before opening output file
* added convenience cmake-macro xacro_add_target()
  to auto-generate xacro-processed files
* added cmake status message before launching xacro
  (xacro might run for quite a while)
* fetch xacro --deps errors at report them as a warning
* simplified deprecation message
* added missing return statement
* removed obsolete math import
  left over from deaaae2c69edd7d5e185eeb098c1521d8711608b
* install xacro.py again (for backwards compatibility)
  usage of xacro.py issues a deprecation warning
* simplified scripts/xacro - removed xacro.py
  - made run script "scripts/xacro" and install process follow standards
  - removed xacro.py
  - added dependencies to setup.py
  Having the binaries xacro and xacro.py installed side by side causes
  problems, because xacro.py is wrongly taken as the module.
  This was avoided by the rather complex filtering of the sys.path.
  Switched to ROS standard now, using a binary script called "xacro".
* changed tests to use the whole xacro processing pipeline
  utilizing the modularization of main() from previous commit
  This simplifies several existing tests, especially these using files on disk.
* split main() into process_cli_arg(), parse(), process_doc()
* extended cmake macro xacro_add_xacro_file()
  - handle INORDER option
  - handle REMAP arguments
  - create absolute input file names automatically
  usage: xacro_add_xacro_file(input output INORDER REMAP ...)
* stripped new unit tests to essential xml snippets
* merged pull request `#68 <https://github.com/ros/xacro/issues/68>`_: eval properties assigned from <arg> tags as literals
  In the following example:
  <xacro:arg name="val" default="0.5"/>
  <xacro:property name="val" value="$(arg val)"/>
  ${val} was not evaluated as a number, but as string only.
  Thus numerical expressions failed with an exception.
* factored out get_boolean_value()
* <xacro:arg> needs to be fully specified
* (handling <arg> tags (without xacro ns-prefix) disabled native <arg> tags)
  add test for eating launch parameter arguments
  remove check for "arg" parameter.
  move new test function to bottom of source
* added unit tests for evaluation of list, tuple, and dict literals
* fixed some code style issues
* fixed string-isinstance checks (for python 3 compatibility)
* do not evaluate list, dict, tuple expressions as literals (without ${} syntax)
* added dict to list of known global symbols
* focused global_symbols definition in the beginning of the file
  added some basic python symbols: list, str, float, int and map
  allowing some basic computation
* tuning performance: instantiate QuickLexer's regexps only once
* Contributors: Robert Haschke, Martin Pecka, Mike O'Driscoll, Morgan Quigley

1.10.1 (2015-04-01)
-------------------
* improved error handling and more descriptive error messages
* correctly raise a XacroException on invalid, i.e. non-boolean, <xacro:if> expressions.
  (removed left-over debugging code, added test case)
* raise an exception on undefined, but used macros
  Using the syntax <xacro:macroname/> should raise an exception if
  macroname is not defined. Added appropriate code and a test case.
* fixed bookkeeping in lazy evaluation
  switch Table.unevaluated from list to set to avoid multiple key entries
* fix formatting of changelog
* Contributors: Robert Haschke

1.10.0 (2015-03-13)
-------------------
* security measure: forbid access to __builtins__ in expressions
* literal evaluation should only consider literals, but no expressions use ast.literal_eval()
* removed eval() from xacro:if evaluation
* back to string comparison to handle (lowercase) true and false
* add test case for equality expressions in <xacro:if>
* add test case for math function usage
* python based evaluation of expressions
  - replaced handle_expr with python-internal eval() call
  - care has been taken to resolve variables recursively on demand (in Table.__getitem__)
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
  - added function text_matches (normalizing consecutive whitespace to a single space)
  - added some new unit tests
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
