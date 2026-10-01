# A Fitted Delay Table Never Reached The Shaper

## Claim

Every profile carrying a custom netem delay distribution failed at arming time,
on every image this project ships, since the feature was added. The runner
installed the table as `/usr/lib/tc/<name>.dist`; iproute2 compiles its library
directory in, and on a multiarch Debian or Ubuntu build that directory is
`/usr/lib/<triplet>/tc`. `/usr/lib/tc` does not exist there at all, so the copy
created it, succeeded, and put the file where nothing reads it:

```
rosotacom: error: network shaping command failed in …_com_to_b:
  tc qdisc replace dev eth0 root handle 10: netem delay 25ms 18ms
    distribution field-20260817 loss gemodel 0.102% 2.5% 2.7% 0.098%
  -> No distribution data for field-20260817
     (/usr/lib/x86_64-linux-gnu/tc//field-20260817.dist: No such file or directory)
```

The error names the directory `tc` wanted, one line after the runner printed
that it had installed the table somewhere else, and the two paths were never
compared. What made it survive is that no committed finding needed the table:
`field-20260817-case3-delay-jam`, the one field profile a published run used,
carries `rate` and `delay` and no `distribution`. The three profiles that do
carry one — `-steady`, `-events`, `-soggy` — are exactly the ones nothing had
run end to end.

The consequence is not a wrong number anywhere. It is that the drive-derived
delay distribution, whose whole purpose is that a normal jitter model
under-tails this link by about a factor of two, could not be used, so every
replay of the 2026-08-17 link either omitted its tail or fell back to a builtin
shape.

## Setup

- Host pair / topology: one host, the packaged local benchmark rig — two
  communication containers on their own Docker network, `tc` applied inside
  each container's own netns (`--sudo-mode container`), no host privileges.
- Session: `bench_1_1_capacity` from the packaged example project, one
  `a->b:/bench_capacity` stream at 12000 B and 10 Hz.
- rosotacom SHA: present up to `25251cb`, fixed in this commit. The defect
  reaches back to the commit that introduced custom distribution tables.
- Profile: `field-20260817-events` from the packaged example set — the one that
  names `distribution: field-20260817` with
  `distribution_file: dist/field-20260817.dist`.
- Seed policy: not applicable — the failure is present/absent at arming time,
  before any traffic is offered.

## Evidence

Evidence grade: reproduction on packaged material, plus the image's own answer
about where its `tc` looks.

The image is asked rather than assumed:

```bash
docker run --rm --entrypoint sh <communication image> -c \
  'tc -V; ls -d /usr/lib/tc /usr/lib/*-linux-gnu*/tc; strings $(command -v tc) | grep /tc/$'
```

```
tc utility, iproute2-6.1.0, libbpf 1.3.0
ls: cannot access '/usr/lib/tc': No such file or directory
/usr/lib/x86_64-linux-gnu/tc
/usr/lib/x86_64-linux-gnu/tc/
```

The directory the runner wrote to did not exist before the runner created it;
the directory `tc` reads already held iproute2's own `normal.dist` and
`pareto.dist`. That asymmetry is the fix: the directory holding those tables is
proof, where a triplet derived from the host would be a guess — the containers
need not share the host's architecture.

Before and after, same command, same profile:

| | arming | 30 s probe at 12 kB / 10 Hz |
|---|---|---|
| before | `No distribution data for field-20260817` | run aborted |
| after | table installed in `/usr/lib/x86_64-linux-gnu/tc` | 300 of 306 delivered, 1.96 % loss, p50 55.4 ms, p95 102.5 ms |

The delivered numbers are the second half of the evidence: 1.96 % against the
fitted drive's own 1.9–2.7 %, and a p95 the builtin `normal` distribution does
not produce at this mean and standard deviation.

Verification: automated:
`pytest tests/unit/test_network_profiles.py -k tc_lib_dir` builds the resolver's
shell program against a temporary root and asserts all three rungs — a directory
carrying iproute2's own tables wins over one that merely exists, and an image
with neither gets `/usr/lib/tc` created. Manual, for the effect itself: run
`rosotacom benchmark probe --profile field-20260817-events --size 12000
--rate-hz 10 --duration 30 --sudo-mode container` from the packaged example
project; the run must reach a delivery figure rather than fail at arming, and
the line it prints must name the directory that already holds `normal.dist`.

## Status

confirmed, 2026-08-30.

Fixed by resolving the directory inside the container instead of hard-coding
it, in three rungs: a candidate that already carries one of iproute2's own
tables, a candidate that merely exists, and finally `/usr/lib/tc`, created, for
an image that has neither.

## Publication notes

Two things are worth reporting, and the smaller one is the bug.

The first is about what a green run proves. Every layer here reported success —
the copy succeeded, the directory was created, the runner printed the path it
had written to — and the only component that knew the path was wrong was the
one that read it, one line later, in a message nobody was comparing against the
line above it. An installation step that does not verify against the consumer's
own view of where it looks is not an installation step.

The second is for anyone replaying a cellular trace through `netem`. The reason
this defect matters is that the builtin `normal` distribution is the wrong
shape for this link: at the 2026-08-17 drive's mean and standard deviation it
predicts a p99 about half of what was measured, which is the half a deadline
cares about. A trace-driven emulation that reports its delay model as
"delay X ± Y, normal" is therefore not reproducing the tail it was fitted to,
and the table that would have fixed it was, here, silently not in use.
