## Loading the DSEC sample data

Build and run the standalone loader from `cpp-realtime`:

```sh
cd cpp-realtime
cmake -S . -B build
cmake --build build --target dsec-app
./build/dsec-app
```

By default it loads `../sampledata/events_first20.h5`,
`../sampledata/rectify_map.h5`, and
`../sampledata/interlaken_00_b_first20.csv`. The trimmed event file contains
events through timestamp `51658100469` and the CSV contains its header plus the
first 20 intervals. The event arrays require approximately 9 GiB of memory.
Alternate files can be provided as positional arguments in that order.

The original, untrimmed `events.h5` uses the Blosc HDF5 filter. To load that
file instead, install a compatible HDF5 Blosc plugin and point
`HDF5_PLUGIN_PATH` at its plugin directory. For example, with the Python
`hdf5plugin` package:

```sh
python3 -m pip install hdf5plugin
export HDF5_PLUGIN_PATH="$(python3 -c 'import hdf5plugin; print(hdf5plugin.PLUGINS_PATH)')"
```
