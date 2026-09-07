# Testing bounded remote updates

With the regular Usteer build dependencies installed:

```sh
cmake -S . -B build-test -DBUILD_TESTING=ON
cmake --build build-test
ctest --test-dir build-test --output-on-failure
```

For ASan/UBSan, also pass these options when configuring:

```sh
-DCMAKE_C_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer" \
-DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined"
```

The test uses the unchanged parser from `parse.c`. It checks byte-identical
forwarding of small messages, limits exactly at and just below the message
size, and 1,001 payload limits from 200 to 1,200 bytes with 300 stations
across two nodes plus a third, empty node. Station records must not be lost
or duplicated. Metadata, unknown node fields and the input buffer must be
preserved. Further cases cover indivisible oversized metadata in the last
node, oversized station records, small MTUs, host-only updates and send
errors. Two tests send and receive messages through actual IPv4/IPv6
loopback UDP sockets; IPv6 loopback must be available in the test environment.

`remote-socket` also exercises the actual send path with mocked system
calls: no MTU query before socket registration or after removal, valid
descriptor 0 and a controlled MTU query failure. The initial periodic send
is started only after socket initialization.

## Protocol and limitations

Splitting uses only existing message fields. Each chunk contains the same
host metadata and sequence number as the logical update. The existing
receiver does not use the sequence number to suppress duplicates and
updates station records individually; missing stations are not deleted
because an update contains only a subset. No new receive logic or
simultaneous upgrade of all peers is required. Parser and data tests
support this, but do not replace a multi-AP deployment test.

The UDP payload limit is at most 1,200 bytes and is reduced further for
smaller local interface MTUs, accounting for IPv4/IPv6 and UDP headers.
This is not general path-MTU discovery for tunnels or routed networks.
The entire message is checked for splittability before sending any chunk.
An indivisible oversized metadata block or station record produces
`EMSGSIZE` and a logged send error, not truncation or an oversized fallback.
A later socket send error can still leave a partially received update,
as with ordinary UDP loss; subsequent periodic updates continue normally.

The patch is intended to avoid IP fragmentation of Usteer updates on the
intended LAN interfaces. It does not establish a fix for unrelated radio
or client connectivity problems.
