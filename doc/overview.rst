.. _overview:

Overview
========

`Pyros`_ is connecting multiple processes together, from different "multiprocess environments" while assuming the strict minimum about these environments.
The main focus is interfacing Python multiprocess environment to `ROS`_ multiprocess environment, but we should work towards solving any kind of multiprocess bridging.

A process is supposed to be one program, isolated from all others, so how can you write a program that interface two programs without using one of the underlying mutliprocess system ?
It s a chicken and egg problem : to be able to have one set of processes talk to another set of processes, you need to have a multiprocess framework that enable interprocess communication.

`Pyros`_ solves that problem by providing his own multiprocess model, that strives to be minimal, yet complete enough to be able to solve any problem that is traditionally solved with `imperative`_ or `functional`_ `paradigms`_.
All you might want to be able to do with a multiprocess software should be doable with `Pyros`_.

Standing on the shoulder of giants
----------------------------------

We are leveraging existing technologies, and concepts :

- Multiprocess is done by `python.multiprocessing`_ . Eventually this can be extended to support other concurrency models, following a similar API :

  - Threads (with `python.threading`_) (TO COME)
  - monothread (TO COME : Entities as understood from `Entity Component Systems`_ design)

- Communication is handled by `pyzmq`_ and can be done via multiple channels :

  - OS pipes
  - network sockets
  - shared memory (TO COME)

- Serialization is done by `pickle`_, with planning to plug any other serialization library that can support the same pickle API:

  - `jsonpickle`_ (TO COME)
  - etc.

Assumptions
-----------
We are working based on the assumption that there is a minimal general-purpose distributed computing model, able to solve all traditionally solvable computational problems.
While this model is not clearly define theoretically (if you know one, let `me`_ know !), a big part of the work is identifying which concepts are present in the many multiprocess framework existing and extract the lowest common denominator from them.

`Functional programming`_ can completely abstract the underlying realities of computers for the sake of nice algorithm.
While it can be useful in some cases, most real world applications still need to care about how they run, and developers, as humans, naturally think in the "Subject Action Object" form.
On a running system, the process is the subject, and therefore abstracting it will remove a concept that is useful for most developer while thinking about his application.

Currently our multiprocessing model is made of :

- Nodes : Think web servers, think one command line process reading from a unix pipe and writing into a file, think `stream processing`_, think `dataflow programming`_.

- Services : think `REST`_ API backend, think `remote procedure call`_, think `subroutine`_, synchronous or `asynchronous`_ from the client point of view.

- Params : these are here to be able to interface params from ROS. But ultimately there should be some concept of global state, a set of `global variables`_.

While this is not strictly needed ( functional languages dont have that ), it is widely used, and not having it can make some simple task more complicated than needed.
The current plan is to provide some resilient distributed global state by implementing a `consensus`_ algorithm like  `raft`_ in multiprocessing python. Using it in an appliation is not mandatory, and not optimum, but it will be stable and usable from any node, without single point of failure.

- Something else... : There is a concept missing here, that is the complementary of a service.

Something that always broadcast data, where loss is acceptable, and that can reach fast transfer rate.
ROS has Topics. But Topics are actually not really useful, since you always need to worry about the endpoints,
the connection ends from publisher side or from subscriber side. We need a concept that can interface with topics,
yet that doesnt require people to think about connection endpoints.
This is probably the type of communication used for stream/dataflow programming, however most implementation do that inside one process and we need to do that between processes.

Note : This will evolve as the software, and our understanding of complex multiprocess systems evolves.



.. include:: weblinks.rst