# generated from catkin/cmake/em/order_packages.cmake.em
@{
import os
try:
    from catkin_pkg.cmake import get_metapackage_cmake_template_path
except ImportError as e:
    raise RuntimeError('ImportError: "from catkin_pkg.cmake import get_metapackage_cmake_template_path" failed: %s\nMake sure that you have installed "catkin_pkg", it is up to date and on the PYTHONPATH.' % e)
try:
    from catkin_pkg.topological_order import topological_order
except ImportError as e:
    raise RuntimeError('ImportError: "from catkin_pkg.topological_order import topological_order" failed: %s\nMake sure that you have installed "catkin_pkg", it is up to date and on the PYTHONPATH.' % e)
try:
    from catkin_pkg.package import InvalidPackage
except ImportError as e:
    raise RuntimeError('ImportError: "from catkin_pkg.package import InvalidPackage" failed: %s\nMake sure that you have installed "catkin_pkg", it is up to date and on the PYTHONPATH.' % e)
# vars defined in order_packages.context.py.in
try:
    ordered_packages = topological_order(os.path.normpath(source_root_dir), whitelisted=whitelisted_packages, blacklisted=blacklisted_packages, underlay_workspaces=underlay_workspaces)
except InvalidPackage as e:
    print('message(FATAL_ERROR "%s")' % ('%s' % e).replace('"', '\\"'))
    ordered_packages = []
fatal_error = False
}@

set(CATKIN_ORDERED_PACKAGES "")
set(CATKIN_ORDERED_PACKAGE_PATHS "")
set(CATKIN_ORDERED_PACKAGES_IS_META "")
set(CATKIN_ORDERED_PACKAGES_BUILD_TYPE "")
@[for path, package in ordered_packages]@
@[if path is None]@
message(FATAL_ERROR "Circular dependency in subset of packages:\n@package")
@{
fatal_error = True
}@
@[elif package.name != 'catkin']@
list(APPEND CATKIN_ORDERED_PACKAGES "@(package.name)")
list(APPEND CATKIN_ORDERED_PACKAGE_PATHS "@(path.replace('\\','/'))")
list(APPEND CATKIN_ORDERED_PACKAGES_IS_META "@(str('metapackage' in [e.tagname for e in package.exports]))")
list(APPEND CATKIN_ORDERED_PACKAGES_BUILD_TYPE "@(str([e.content for e in package.exports if e.tagname == 'build_type'][0]) if 'build_type' in [e.tagname for e in package.exports] else 'catkin')")
@{
deprecated = [e for e in package.exports if e.tagname == 'deprecated']
}@
@[if deprecated]@
message("WARNING: Package '@(package.name)' is deprecated@(' (%s)' % deprecated[0].content if deprecated[0].content else '')")
@[end if]@
@[end if]@
@[end for]@

@[if not fatal_error]@
@{
message_generators = [package.name for (_, package) in ordered_packages if 'message_generator' in [e.tagname for e in package.exports]]
}@
set(CATKIN_MESSAGE_GENERATORS @(' '.join(message_generators)))
@[end if]@

set(CATKIN_METAPACKAGE_CMAKE_TEMPLATE "@(get_metapackage_cmake_template_path().replace('\\','/'))")
