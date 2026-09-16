# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [5.0.0] - 2026-09-16

### Added

- Add `ex_session.py` to expected executables within Debian package check ([bdec262](https://github.com/AndrejOrsula/pymoveit2/commit/bdec2621c1a9b31e7022ef2d1217c3d6091fa4c2)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Add example for `RobotSession` usage via `connect()` ([f055649](https://github.com/AndrejOrsula/pymoveit2/commit/f0556496b13778d3fa4d16784c218e337efec548)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- CI: Add PyPI release workflow for Python package distribution ([f8a59d8](https://github.com/AndrejOrsula/pymoveit2/commit/f8a59d8d3afc5c1a8152681849f4e155a1133a20)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- CI: Add Docker release workflow ([f39838f](https://github.com/AndrejOrsula/pymoveit2/commit/f39838fcfc03e4c358a231c582bcce173adcb59e)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Add doctor example to help with diagnosing MoveIt 2 setup issues ([3911a84](https://github.com/AndrejOrsula/pymoveit2/commit/3911a846dea44898f15516d05c0d2b27421dad99)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Docker: Add Dockerfile and helper scripts ([9d32e1c](https://github.com/AndrejOrsula/pymoveit2/commit/9d32e1c2c65e6c591111e9bd1ed139c8148bad77)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Add simplified RobotSession manager for MoveIt2 interfaces ([41754d7](https://github.com/AndrejOrsula/pymoveit2/commit/41754d707a651984fb725e11c78c900ea943505b)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Add script for running tests across different ROS 2 distributions ([53ae617](https://github.com/AndrejOrsula/pymoveit2/commit/53ae6178e4fcb3f8205bc187170af3d386656fc1)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Add integration tests ([57b3886](https://github.com/AndrejOrsula/pymoveit2/commit/57b388697b482807c9c4d972ce4666fea0d3e4c2)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Add typecheck test ([3d0060e](https://github.com/AndrejOrsula/pymoveit2/commit/3d0060ef7f5606177d0730cbe5336cd8a6d6fd04)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Add core tests ([31d37b8](https://github.com/AndrejOrsula/pymoveit2/commit/31d37b8f40a96e1894a885ee97d07369871bedbf)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Add Python package manifest and metadata ([de8842c](https://github.com/AndrejOrsula/pymoveit2/commit/de8842c68aee86f93b299a8680c926e0764eb04d)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Add automated robot description discovery ([5c941c4](https://github.com/AndrejOrsula/pymoveit2/commit/5c941c4579f2acbf672d6ec0963415befa34afec)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Gitignore: Add cache and coverage reports ([f82f918](https://github.com/AndrejOrsula/pymoveit2/commit/f82f91854bfd8d83397a22ebf3120aa26c21db71)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Added joints jogging with MoveIt2 servo ([1416cb5](https://github.com/AndrejOrsula/pymoveit2/commit/1416cb58ded8b3babb1c97c7a406e75e46dfcad2)) by [@macmacal](https://github.com/macmacal)

### Changed

- Docker: Update .dockerignore to allow helper scripts ([e0d2194](https://github.com/AndrejOrsula/pymoveit2/commit/e0d2194c6a00278e7a27c306fb7faf2200c6b24f)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Build(deps): bump actions/upload-artifact from 4.6.2 to 7.0.1 ([e5edd27](https://github.com/AndrejOrsula/pymoveit2/commit/e5edd27e2f41cfc19125bb48ad8ed2cb5e210358)) by [@dependabot[bot]](https://github.com/dependabot%5Bbot%5D) in [#135](https://github.com/AndrejOrsula/pymoveit2/pull/135)
- Build(deps): bump actions/download-artifact from 4.3.0 to 8.0.1 ([60842da](https://github.com/AndrejOrsula/pymoveit2/commit/60842da0d67cde728ee098a2bfe02d0db54b9870)) by [@dependabot[bot]](https://github.com/dependabot%5Bbot%5D) in [#137](https://github.com/AndrejOrsula/pymoveit2/pull/137)
- Build(deps-dev): bump mypy from 1.15.0 to 2.3.1 in /.ci ([6de83c7](https://github.com/AndrejOrsula/pymoveit2/commit/6de83c72c0ae2b1ffaf9eb86da072cf69d956399)) by [@dependabot[bot]](https://github.com/dependabot%5Bbot%5D) in [#134](https://github.com/AndrejOrsula/pymoveit2/pull/134)
- Build(deps-dev): bump coverage from 7.6.10 to 7.16.0 in /.ci ([31369e5](https://github.com/AndrejOrsula/pymoveit2/commit/31369e5d5e3befadc0aef8ccee47d747db4ca691)) by [@dependabot[bot]](https://github.com/dependabot%5Bbot%5D) in [#136](https://github.com/AndrejOrsula/pymoveit2/pull/136)
- Update CHANGELOG for 5.0.0 ([76eb79c](https://github.com/AndrejOrsula/pymoveit2/commit/76eb79c9f67d2f9e2df530196d51e3994ab509ff)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Deprecate `max_step` parameter in favor of `cartesian_max_step` ([d101370](https://github.com/AndrejOrsula/pymoveit2/commit/d101370190295738528c1efb3f66b710d7afcdf0)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Update documentation for the 5.0.0 release ([d83a4c3](https://github.com/AndrejOrsula/pymoveit2/commit/d83a4c3487b88c9222fcc2702a124cefa42843a4)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- CI: Update Debian package build workflow ([69c4801](https://github.com/AndrejOrsula/pymoveit2/commit/69c480123d7304690e22ae4c5afbaf91dd0e207f)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- CI: Update Dependabot automation ([99581a9](https://github.com/AndrejOrsula/pymoveit2/commit/99581a98e2e97ef37dd69bfe52eeac812b24d8b3)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- CI: Automate testing and codecov reporting across ROS 2 distributions ([fb66745](https://github.com/AndrejOrsula/pymoveit2/commit/fb6674577d1d102dbd71ad66c6741f541257ed79)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- CI: Update pre-commit workflow ([cbf470f](https://github.com/AndrejOrsula/pymoveit2/commit/cbf470f98bac7c35d41bda402515af2888ce5c47)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Update examples ([0371a5d](https://github.com/AndrejOrsula/pymoveit2/commit/0371a5d81677ccd35aafc6921113b393d4930f91)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Update ROS 2 package manifest ([7b2824b](https://github.com/AndrejOrsula/pymoveit2/commit/7b2824b9a361a4b6d6a24c934f1d844e7485aa21)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Standardize MoveIt2Servo across different ROS 2 distributions ([b055206](https://github.com/AndrejOrsula/pymoveit2/commit/b055206a56ea70e447805884ee82fd236e08f2ac)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Refactor the unified gripper interface with lifecycle management and joint state validation ([6654cd6](https://github.com/AndrejOrsula/pymoveit2/commit/6654cd696d5c1585eabc5cb0c51d9ed42cefac7c)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Refactor MoveIt2 interface with lifecycle management and error handling ([2643626](https://github.com/AndrejOrsula/pymoveit2/commit/26436266b87ece569c2bdbe89e0d5503a20ce3be)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- License: Update copyright year to 2026 ([0fcccf1](https://github.com/AndrejOrsula/pymoveit2/commit/0fcccf168b63929369da1f5eb71eb80bbc66d581)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Pre-commit: Update hooks ([c2bbfb4](https://github.com/AndrejOrsula/pymoveit2/commit/c2bbfb45f7cbc23f9008289af0e08c6a627a5165)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Pre-commit: Update setup script usage message ([3b9d727](https://github.com/AndrejOrsula/pymoveit2/commit/3b9d727af7f6594791411c3b2b398b575904f511)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Build(deps): bump actions/setup-python from 6 to 7 ([149c164](https://github.com/AndrejOrsula/pymoveit2/commit/149c164dfa9ca7df3b235bed4da6af79992b0351)) by [@dependabot[bot]](https://github.com/dependabot%5Bbot%5D) in [#132](https://github.com/AndrejOrsula/pymoveit2/pull/132)
- Build(deps): bump actions/checkout from 6 to 7 ([b1bcfe3](https://github.com/AndrejOrsula/pymoveit2/commit/b1bcfe35406842dc2f3757a29096ae91de5b6e89)) by [@dependabot[bot]](https://github.com/dependabot%5Bbot%5D) in [#131](https://github.com/AndrejOrsula/pymoveit2/pull/131)
- Build(deps): bump dependabot/fetch-metadata from 2 to 3 ([680bdf9](https://github.com/AndrejOrsula/pymoveit2/commit/680bdf9e0381bb2abac78c7835a115bcd1086d5f)) by [@dependabot[bot]](https://github.com/dependabot%5Bbot%5D) in [#129](https://github.com/AndrejOrsula/pymoveit2/pull/129)
- Apply the custom start joint state to the request ([900c137](https://github.com/AndrejOrsula/pymoveit2/commit/900c137499ec70d5a5cb216e24d88e91c7a508fc)) by [@AndrejOrsula](https://github.com/AndrejOrsula) in [#127](https://github.com/AndrejOrsula/pymoveit2/pull/127)
- Avoid blocking update planning scene ([64ce749](https://github.com/AndrejOrsula/pymoveit2/commit/64ce749e13dbee93da9cfbddddf695656b6faaae)) by [@AndrejOrsula](https://github.com/AndrejOrsula) in [#126](https://github.com/AndrejOrsula/pymoveit2/pull/126)
- Docs: Update naming from Ignition to Gazebo ([3568a98](https://github.com/AndrejOrsula/pymoveit2/commit/3568a987023ac988b164002729431c74b372ca5e)) by [@AndrejOrsula](https://github.com/AndrejOrsula) in [#125](https://github.com/AndrejOrsula/pymoveit2/pull/125)
- Use context managers for mutex ([732cbc5](https://github.com/AndrejOrsula/pymoveit2/commit/732cbc579c119b8a4b2db9095455b0b7b26dcda3)) by [@mfinean](https://github.com/mfinean) in [#113](https://github.com/AndrejOrsula/pymoveit2/pull/113)
- Merge pull request #124 from christian-rauch/ci_allow_concurrency ([d5fc691](https://github.com/AndrejOrsula/pymoveit2/commit/d5fc6915a1434829fd2d6cd5d513fa64fa205557)) by [@christian-rauch](https://github.com/christian-rauch) in [#124](https://github.com/AndrejOrsula/pymoveit2/pull/124)
- Allow concurrent CI jobs ([9d653b1](https://github.com/AndrejOrsula/pymoveit2/commit/9d653b15500520620a0d5a0803febbe50dedf792)) by [@christian-rauch](https://github.com/christian-rauch)
- Merge pull request #117 from macmacal/bugfix/servo_topic_name_typo ([36b6ca7](https://github.com/AndrejOrsula/pymoveit2/commit/36b6ca76b4a51946e48c8aa1e652233668e9a55a)) by [@christian-rauch](https://github.com/christian-rauch) in [#117](https://github.com/AndrejOrsula/pymoveit2/pull/117)
- Merge remote-tracking branch 'origin/main' into bugfix/servo_topic_name_typo ([083bda2](https://github.com/AndrejOrsula/pymoveit2/commit/083bda2ae40638659b83481a5edb4206616a18bf)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Merge pull request #118 from macmacal/feature/add_servo_jogging ([79088ac](https://github.com/AndrejOrsula/pymoveit2/commit/79088ac4e18011697d05bbbadd42d9ae504c2245)) by [@christian-rauch](https://github.com/christian-rauch) in [#118](https://github.com/AndrejOrsula/pymoveit2/pull/118)
- Merge remote-tracking branch 'origin/main' into feature/add_servo_jogging ([df707fc](https://github.com/AndrejOrsula/pymoveit2/commit/df707fc72172b35519efd79d5ab6a0d10ccb617f)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- CI: Disable pull_request trigger for the build workflow ([e581b73](https://github.com/AndrejOrsula/pymoveit2/commit/e581b733b53eb3aa0782972c33accb905b9257d9)) by [@AndrejOrsula](https://github.com/AndrejOrsula) in [#121](https://github.com/AndrejOrsula/pymoveit2/pull/121)
- Build(deps): bump actions/checkout from 5 to 6 ([e40c904](https://github.com/AndrejOrsula/pymoveit2/commit/e40c904ac267a01935e4b0abee0150d54e319825)) by [@dependabot[bot]](https://github.com/dependabot%5Bbot%5D) in [#120](https://github.com/AndrejOrsula/pymoveit2/pull/120)
- Merge pull request #115 from AndrejOrsula/ci_run_example ([8c562a4](https://github.com/AndrejOrsula/pymoveit2/commit/8c562a4133665dcacdc5e2119e15fe0e670020be)) by [@christian-rauch](https://github.com/christian-rauch) in [#115](https://github.com/AndrejOrsula/pymoveit2/pull/115)
- Handle ExternalShutdownException during wait for joint states ([20695be](https://github.com/AndrejOrsula/pymoveit2/commit/20695be5d2fbab95f12f353881d82a4900b27210)) by [@christian-rauch](https://github.com/christian-rauch)
- Run 'ex_pose_goal' example ([7b74438](https://github.com/AndrejOrsula/pymoveit2/commit/7b744381c9c77d83c5217e19d67c7364917ff0b7)) by [@christian-rauch](https://github.com/christian-rauch)
- Replace deprecated 'warn' with 'warning' ([efe253a](https://github.com/AndrejOrsula/pymoveit2/commit/efe253a73b3d219de178fdd1b831f3151af63e18)) by [@christian-rauch](https://github.com/christian-rauch)
- Replace internal '\_logger' member with public 'get_logger()' method ([c63e58f](https://github.com/AndrejOrsula/pymoveit2/commit/c63e58f7cca925b1e3949cbb8dfe7aecfeece159)) by [@christian-rauch](https://github.com/christian-rauch)
- Install ROS 2 CLI Packages ([db8737c](https://github.com/AndrejOrsula/pymoveit2/commit/db8737c5bfe65c7d9c22638c44774bdd6690ae1e)) by [@christian-rauch](https://github.com/christian-rauch)

### Fixed

- CI: Fix test for documenting script usage ([c5a30ad](https://github.com/AndrejOrsula/pymoveit2/commit/c5a30ad1f9d0f97f252ef9a35038171be7945299)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Fix dockerignore matches across tests ([747ea2a](https://github.com/AndrejOrsula/pymoveit2/commit/747ea2a2f7cfc0bb84eb8f20c20c558213d30ed2)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- CI: Fix path to scripts and set coverage threshold to 65% ([5de2535](https://github.com/AndrejOrsula/pymoveit2/commit/5de253595b7d835a9c4d2a06479ef7dfff55950f)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Fix type error in mesh upload ([b25c9a6](https://github.com/AndrejOrsula/pymoveit2/commit/b25c9a6c294ef433b07b1eafc42a93d08d8612e5)) by [@grahas](https://github.com/grahas) in [#133](https://github.com/AndrejOrsula/pymoveit2/pull/133)
- Fixed topic path for servo commands ([5c6b24b](https://github.com/AndrejOrsula/pymoveit2/commit/5c6b24bb2fb70a4ad07e71600be62412ec312c61)) by [@macmacal](https://github.com/macmacal)

### Removed

- Remove non-existing 'master' branch ([cce4671](https://github.com/AndrejOrsula/pymoveit2/commit/cce46716318d02e010b268466cd73dad6650daeb)) by [@christian-rauch](https://github.com/christian-rauch)

## New Contributors

- [@grahas](https://github.com/grahas) made their first contribution in [#133](https://github.com/AndrejOrsula/pymoveit2/pull/133)
- [@macmacal](https://github.com/macmacal) made their first contribution

## [4.2.0] - 2025-10-31

### Added

- Add a new namespace parameter to MoveIt2Servo ([8b79149](https://github.com/AndrejOrsula/pymoveit2/commit/8b791491b497822b45b4e4b41af18e6d8abf6c53)) by [@Flamethr0wer](https://github.com/Flamethr0wer) in [#102](https://github.com/AndrejOrsula/pymoveit2/pull/102)
- Add a method for setting the workspace parameters ([a89b816](https://github.com/AndrejOrsula/pymoveit2/commit/a89b816a94bd1daed0995a1364686a0270589373)) by [@mfinean](https://github.com/mfinean) in [#106](https://github.com/AndrejOrsula/pymoveit2/pull/106)
- CI: Add build workflow ([c2dda9e](https://github.com/AndrejOrsula/pymoveit2/commit/c2dda9e97d96586fd6071b7d26d3d236d8c252d7)) by [@AndrejOrsula](https://github.com/AndrejOrsula) in [#97](https://github.com/AndrejOrsula/pymoveit2/pull/97)

### Changed

- Merge pull request #114 from AndrejOrsula/rel_412 ([5f97078](https://github.com/AndrejOrsula/pymoveit2/commit/5f970783bbc628aeb38daa4eae62bc83364536ec)) by [@christian-rauch](https://github.com/christian-rauch) in [#114](https://github.com/AndrejOrsula/pymoveit2/pull/114)
- Update the release version in the package.xml ([dea501f](https://github.com/AndrejOrsula/pymoveit2/commit/dea501f120b98456aaa58ccd0fd9c191e963fc2e)) by [@christian-rauch](https://github.com/christian-rauch)
- Build(deps): bump actions/setup-python from 5 to 6 ([f2fc1c4](https://github.com/AndrejOrsula/pymoveit2/commit/f2fc1c4d59752581538c3f493a16866cf72a6a39)) by [@dependabot[bot]](https://github.com/dependabot%5Bbot%5D) in [#112](https://github.com/AndrejOrsula/pymoveit2/pull/112)
- Apply the custom start joint state to the request ([2caf522](https://github.com/AndrejOrsula/pymoveit2/commit/2caf522ad1d0501759774385ed17544ddc6d027a)) by [@mfinean](https://github.com/mfinean) in [#109](https://github.com/AndrejOrsula/pymoveit2/pull/109)
- Expose link name argument for computing IK ([bbf125a](https://github.com/AndrejOrsula/pymoveit2/commit/bbf125a2f2ef1f23501a986ddf7f052116be57d6)) by [@Zarnack](https://github.com/Zarnack) in [#105](https://github.com/AndrejOrsula/pymoveit2/pull/105)
- Build(deps): bump actions/checkout from 4 to 5 ([584f183](https://github.com/AndrejOrsula/pymoveit2/commit/584f183fe9d444773a1354d6af0e271af76091cd)) by [@dependabot[bot]](https://github.com/dependabot%5Bbot%5D) in [#108](https://github.com/AndrejOrsula/pymoveit2/pull/108)
- Comment out trimesh pip dependency for bloom package ([12ca304](https://github.com/AndrejOrsula/pymoveit2/commit/12ca3044195973a2b82dced5b5a7b04f4a2ade99)) by [@christian-rauch](https://github.com/christian-rauch) in [#99](https://github.com/AndrejOrsula/pymoveit2/pull/99)
- Revert #94, add trimesh to ROS dependencies, and remove sync param from `reset_controller()` methods ([f418cd9](https://github.com/AndrejOrsula/pymoveit2/commit/f418cd9639a6322de0133b8243105ac1aa0d4621)) by [@christian-rauch](https://github.com/christian-rauch) in [#96](https://github.com/AndrejOrsula/pymoveit2/pull/96)
- Always specify executor when spinning the node ([4797ed5](https://github.com/AndrejOrsula/pymoveit2/commit/4797ed5b0b89e602f184459b99f08257f78f319d)) by [@alonborn](https://github.com/alonborn) in [#94](https://github.com/AndrejOrsula/pymoveit2/pull/94)

## New Contributors

- [@christian-rauch](https://github.com/christian-rauch) made their first contribution in [#114](https://github.com/AndrejOrsula/pymoveit2/pull/114)
- [@mfinean](https://github.com/mfinean) made their first contribution in [#109](https://github.com/AndrejOrsula/pymoveit2/pull/109)
- [@Flamethr0wer](https://github.com/Flamethr0wer) made their first contribution in [#102](https://github.com/AndrejOrsula/pymoveit2/pull/102)
- [@alonborn](https://github.com/alonborn) made their first contribution in [#94](https://github.com/AndrejOrsula/pymoveit2/pull/94)

## [4.1.1] - 2025-03-04

### Fixed

- Fix rclpy import and spin issues ([4b1260a](https://github.com/AndrejOrsula/pymoveit2/commit/4b1260aeada4268fd92770e889be4a9bf59fbcf8)) by [@andrewbowman23](https://github.com/andrewbowman23) in [#88](https://github.com/AndrejOrsula/pymoveit2/pull/88)

## New Contributors

- [@andrewbowman23](https://github.com/andrewbowman23) made their first contribution in [#88](https://github.com/AndrejOrsula/pymoveit2/pull/88)

## [4.1.0] - 2025-03-04

### Added

- Add UR robot + generalize example to every robot ([b9b82b8](https://github.com/AndrejOrsula/pymoveit2/commit/b9b82b8d397da942971c584f3f1eab58445a9163)) by [@m0rph03nix](https://github.com/m0rph03nix) in [#76](https://github.com/AndrejOrsula/pymoveit2/pull/76)
- Add function to move gripper to a specific position ([afd30fa](https://github.com/AndrejOrsula/pymoveit2/commit/afd30fa01724ba890599ae6143586d6694ff2319)) by [@ycheng517](https://github.com/ycheng517) in [#70](https://github.com/AndrejOrsula/pymoveit2/pull/70)

### Changed

- Feature/string error codes ([b8dde85](https://github.com/AndrejOrsula/pymoveit2/commit/b8dde8584c82615c864be295b77d532251d37364)) by [@blooop](https://github.com/blooop) in [#83](https://github.com/AndrejOrsula/pymoveit2/pull/83)
- [Bugfix] Copy mesh before scaling if it was passed in as a parameter ([640bb85](https://github.com/AndrejOrsula/pymoveit2/commit/640bb856f8af3d690b9c55ed3844647287a8a1f7)) by [@amalnanavati](https://github.com/amalnanavati) in [#80](https://github.com/AndrejOrsula/pymoveit2/pull/80)
- Pre-commit: Update hooks ([2d684e2](https://github.com/AndrejOrsula/pymoveit2/commit/2d684e2a68ec3ce937f1c3c3321ff5ba72034fd8)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Initialize parameters before ROS2 callbacks ([60cc171](https://github.com/AndrejOrsula/pymoveit2/commit/60cc171b3b75be139411542604cdb52a9275239d)) by [@amalnanavati](https://github.com/amalnanavati) in [#74](https://github.com/AndrejOrsula/pymoveit2/pull/74)
- Build(deps): bump pre-commit-ci/lite-action from 1.0.3 to 1.1.0 ([3ecad18](https://github.com/AndrejOrsula/pymoveit2/commit/3ecad18223aef2d809599c0180223e337a343e34)) by [@dependabot[bot]](https://github.com/dependabot%5Bbot%5D) in [#77](https://github.com/AndrejOrsula/pymoveit2/pull/77)
- Build(deps): bump pre-commit-ci/lite-action from 1.0.2 to 1.0.3 ([704b610](https://github.com/AndrejOrsula/pymoveit2/commit/704b610125f23a5853fee5c69f9602b4394203e2)) by [@dependabot[bot]](https://github.com/dependabot%5Bbot%5D) in [#75](https://github.com/AndrejOrsula/pymoveit2/pull/75)
- Allow clearing all collision objects ([d38a653](https://github.com/AndrejOrsula/pymoveit2/commit/d38a6536670b607faf19e516d68a112fe6348ce1)) by [@amalnanavati](https://github.com/amalnanavati) in [#69](https://github.com/AndrejOrsula/pymoveit2/pull/69)

### Fixed

- Fix potential deadlock and wait until service is available ([1f030f6](https://github.com/AndrejOrsula/pymoveit2/commit/1f030f6072dac16914c91d1368926f182ea5f1fc)) by [@belalhmedan90](https://github.com/belalhmedan90) in [#87](https://github.com/AndrejOrsula/pymoveit2/pull/87)

## New Contributors

- [@belalhmedan90](https://github.com/belalhmedan90) made their first contribution in [#87](https://github.com/AndrejOrsula/pymoveit2/pull/87)
- [@blooop](https://github.com/blooop) made their first contribution in [#83](https://github.com/AndrejOrsula/pymoveit2/pull/83)
- [@m0rph03nix](https://github.com/m0rph03nix) made their first contribution in [#76](https://github.com/AndrejOrsula/pymoveit2/pull/76)
- [@ycheng517](https://github.com/ycheng517) made their first contribution in [#70](https://github.com/AndrejOrsula/pymoveit2/pull/70)

## [4.0.0] - 2024-05-10

### Added

- Add ability to move collision objects ([f362220](https://github.com/AndrejOrsula/pymoveit2/commit/f362220ba44d24e856abb8d3faf861b743b592a5)) by [@amalnanavati](https://github.com/amalnanavati)
- Added setting of cartesian speed and acceleration ([316a433](https://github.com/AndrejOrsula/pymoveit2/commit/316a4330373c3c91792af188ab662f4db43ba972)) by [@Zarnack](https://github.com/Zarnack)
- Add Dependabot automation ([a4146e6](https://github.com/AndrejOrsula/pymoveit2/commit/a4146e6392e325a629b0b0c6370434865751836c)) by [@AndrejOrsula](https://github.com/AndrejOrsula)

### Changed

- Bump version to 4.0.0 ([7f722fd](https://github.com/AndrejOrsula/pymoveit2/commit/7f722fd3fda6e6c958c150cf7d8ad619119d6262)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Autoupdate pre-commit hooks ([02cd3ed](https://github.com/AndrejOrsula/pymoveit2/commit/02cd3ed77ece60cc859a0bb17c4db10a2dc3f67c)) by [@AndrejOrsula](https://github.com/AndrejOrsula) in [#57](https://github.com/AndrejOrsula/pymoveit2/pull/57)
- Update list of supported ROS 2 distributions ([cfa9cb7](https://github.com/AndrejOrsula/pymoveit2/commit/cfa9cb71b754f94a6ca67b0c3059bd1579918e73)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Allow users to preload a collision mesh ([b11ffc1](https://github.com/AndrejOrsula/pymoveit2/commit/b11ffc19335d3618a4d13d9d9788cbf98deb054d)) by [@amalnanavati](https://github.com/amalnanavati)
- Reinstate Humble Compatibility for `GetCartesianPath` & Expose `planning_scene` To User ([f5f596a](https://github.com/AndrejOrsula/pymoveit2/commit/f5f596a9894be0d8a34d2b410618273f93a20d38)) by [@amalnanavati](https://github.com/amalnanavati)
- Allow Scaling Collision Meshes ([1d705a5](https://github.com/AndrejOrsula/pymoveit2/commit/1d705a5bb8b6b5f8ac88a55caf0a2b87f18a0f06)) by [@amalnanavati](https://github.com/amalnanavati)
- Allow users to set various cartesian path service parameters ([6923f3d](https://github.com/AndrejOrsula/pymoveit2/commit/6923f3d35d7f0444d44134d816734335c33f048f)) by [@amalnanavati](https://github.com/amalnanavati)
- Use `cartesian_speed_limited_link` instead of `cartesian_speed_end_effector_link` when available ([d168d4c](https://github.com/AndrejOrsula/pymoveit2/commit/d168d4c4a1355c9a079928316e9b3d64213d7b18)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Allow users to (dis)allow collisions with an object ([fa2610f](https://github.com/AndrejOrsula/pymoveit2/commit/fa2610f98e0e7988789dc7dead847f7d1c29778f)) by [@amalnanavati](https://github.com/amalnanavati)
- Build(deps): bump dependabot/fetch-metadata from 1 to 2 ([bcd2aa9](https://github.com/AndrejOrsula/pymoveit2/commit/bcd2aa91e04adaf6adb885a460216d483b5c212f)) by [@dependabot[bot]](https://github.com/dependabot%5Bbot%5D) in [#54](https://github.com/AndrejOrsula/pymoveit2/pull/54)
- Build(deps): bump pre-commit/action from 3.0.0 to 3.0.1 ([bd63d26](https://github.com/AndrejOrsula/pymoveit2/commit/bd63d260a2c97c97e7e9b14268b1995a2e269235)) by [@dependabot[bot]](https://github.com/dependabot%5Bbot%5D) in [#51](https://github.com/AndrejOrsula/pymoveit2/pull/51)
- Build(deps): bump pre-commit-ci/lite-action from 1.0.1 to 1.0.2 ([0a3f019](https://github.com/AndrejOrsula/pymoveit2/commit/0a3f01937b14206aec88fbe5b1d8f6ea6d6bd89f)) by [@dependabot[bot]](https://github.com/dependabot%5Bbot%5D) in [#52](https://github.com/AndrejOrsula/pymoveit2/pull/52)

### Fixed

- Fix bug in previous PR ([36b554f](https://github.com/AndrejOrsula/pymoveit2/commit/36b554f7b6d1e5cb6b635d65cbc26fd86010110d)) by [@amalnanavati](https://github.com/amalnanavati)

## New Contributors

- [@Zarnack](https://github.com/Zarnack) made their first contribution
- [@dependabot[bot]](https://github.com/dependabot%5Bbot%5D) made their first contribution in [#54](https://github.com/AndrejOrsula/pymoveit2/pull/54)

## [3.2.0] - 2024-02-27

### Added

- Added Async Forward/Inverse Kinematics ([09ce8d8](https://github.com/AndrejOrsula/pymoveit2/commit/09ce8d8a75afaf74da907b07a3a63e53cde70393)) by [@amalnanavati](https://github.com/amalnanavati) in [#43](https://github.com/AndrejOrsula/pymoveit2/pull/43)
- Add Path Constraints ([cff6677](https://github.com/AndrejOrsula/pymoveit2/commit/cff6677e20d356f69e1611c30bae6281d8e97cfc)) by [@amalnanavati](https://github.com/amalnanavati) in [#42](https://github.com/AndrejOrsula/pymoveit2/pull/42)
- Add asynchronous planning and execution ([ae4c8bc](https://github.com/AndrejOrsula/pymoveit2/commit/ae4c8bcc5bc68db3588f3136f1f8f38fa9a8c514)) by [@amalnanavati](https://github.com/amalnanavati) in [#40](https://github.com/AndrejOrsula/pymoveit2/pull/40)
- CI: Add pre-commit ([6268b99](https://github.com/AndrejOrsula/pymoveit2/commit/6268b9993352b408e268b18856792deecb1bcffa)) by [@AndrejOrsula](https://github.com/AndrejOrsula)

### Changed

- Allow users to set `planner_id` and `pipeline_id` ([8bf71ae](https://github.com/AndrejOrsula/pymoveit2/commit/8bf71aef4ede2b301580c5fe6c8d4a12c48400c4)) by [@amalnanavati](https://github.com/amalnanavati) in [#48](https://github.com/AndrejOrsula/pymoveit2/pull/48)

## [3.1.0] - 2023-11-10

### Added

- Add support for KUKA LBRs ([2bc9a43](https://github.com/AndrejOrsula/pymoveit2/commit/2bc9a4302059f8b5d00923b8842a3bb5ad5cfb2a)) by [@mhubii](https://github.com/mhubii) in [#38](https://github.com/AndrejOrsula/pymoveit2/pull/38)
- Added crane_x7 robot model ([5b35728](https://github.com/AndrejOrsula/pymoveit2/commit/5b35728306665e7f21fe2ab1ee5bb42565c68e12)) by [@MDecarabas](https://github.com/MDecarabas) in [#37](https://github.com/AndrejOrsula/pymoveit2/pull/37)

## New Contributors

- [@mhubii](https://github.com/mhubii) made their first contribution in [#38](https://github.com/AndrejOrsula/pymoveit2/pull/38)
- [@MDecarabas](https://github.com/MDecarabas) made their first contribution in [#37](https://github.com/AndrejOrsula/pymoveit2/pull/37)

## [3.0.0] - 2023-10-04

### Added

- Add PhantomX Pincher to robots ([f4bc6b6](https://github.com/AndrejOrsula/pymoveit2/commit/f4bc6b650082532507a713996459e388eadc1618)) by [@AndrejOrsula](https://github.com/AndrejOrsula) in [#33](https://github.com/AndrejOrsula/pymoveit2/pull/33)

### Changed

- Default `skip_if_noop` of gripper `open()`/`close()` to False ([0bb938c](https://github.com/AndrejOrsula/pymoveit2/commit/0bb938cebe2824db8f23468732c0c92f42bafb7e)) by [@AndrejOrsula](https://github.com/AndrejOrsula) in [#36](https://github.com/AndrejOrsula/pymoveit2/pull/36)
- Simplify usage ([4f00642](https://github.com/AndrejOrsula/pymoveit2/commit/4f0064218ea35e20c53c2a1a1e130b6f601df881)) by [@AndrejOrsula](https://github.com/AndrejOrsula) in [#35](https://github.com/AndrejOrsula/pymoveit2/pull/35)
- Improvements for scene objects and gripper interface ([72ebc8a](https://github.com/AndrejOrsula/pymoveit2/commit/72ebc8a183ece8d7ba62ab560a67f21eead4c42f)) by [@AndrejOrsula](https://github.com/AndrejOrsula) in [#34](https://github.com/AndrejOrsula/pymoveit2/pull/34)

## [2.2.0] - 2023-08-23

### Added

- Add Kinova JACO + MICO definitions to robot ([afb0dbd](https://github.com/AndrejOrsula/pymoveit2/commit/afb0dbd2daee08758c32c8f53d8e1ad2ee1f4ac0)) by [@egordon](https://github.com/egordon) in [#32](https://github.com/AndrejOrsula/pymoveit2/pull/32)
- Add `target_link` param to `plan` ([6973d6c](https://github.com/AndrejOrsula/pymoveit2/commit/6973d6c43833abad947ea372c674e2deff8fd60c)) by [@amalnanavati](https://github.com/amalnanavati) in [#30](https://github.com/AndrejOrsula/pymoveit2/pull/30)
- Add a note about the official `moveit_py` bindings ([29dc040](https://github.com/AndrejOrsula/pymoveit2/commit/29dc040ed5fd8703bb19795474114ada2a79d6b0)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Add LICENSE ([9a623a2](https://github.com/AndrejOrsula/pymoveit2/commit/9a623a2d66479cf79dfa80552909392ca4e37b23)) by [@AndrejOrsula](https://github.com/AndrejOrsula)

### Changed

- [pre-commit.ci] pre-commit autoupdate ([f55428a](https://github.com/AndrejOrsula/pymoveit2/commit/f55428ab7a31887436aa5921979abbefe9e648b8)) by [@pre-commit-ci[bot]](https://github.com/pre-commit-ci%5Bbot%5D) in [#22](https://github.com/AndrejOrsula/pymoveit2/pull/22)
- [pre-commit.ci] pre-commit autoupdate ([60eb340](https://github.com/AndrejOrsula/pymoveit2/commit/60eb3406a87151bbaa7cf1d70f41c9c87d6f7663)) by [@pre-commit-ci[bot]](https://github.com/pre-commit-ci%5Bbot%5D) in [#21](https://github.com/AndrejOrsula/pymoveit2/pull/21)
- [pre-commit.ci] pre-commit autoupdate ([c6c02d9](https://github.com/AndrejOrsula/pymoveit2/commit/c6c02d9d317b3596a1b405045ea236c38a0df44d)) by [@pre-commit-ci[bot]](https://github.com/pre-commit-ci%5Bbot%5D) in [#15](https://github.com/AndrejOrsula/pymoveit2/pull/15)
- Use `frame_id` in `MoveIt2._plan_cartesian_path()` ([15c17c5](https://github.com/AndrejOrsula/pymoveit2/commit/15c17c539bd6a79efc19c3f0e249200eb8148d12)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Clean-up examples ([854beb1](https://github.com/AndrejOrsula/pymoveit2/commit/854beb12a950632dee2b96b6686c65812a55722c)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Update & apply pre-commit hooks ([67f455f](https://github.com/AndrejOrsula/pymoveit2/commit/67f455fad35af0b69c4f754f1311175ac00cb7b0)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Enable execution of bash scripts via symlinks ([20addbf](https://github.com/AndrejOrsula/pymoveit2/commit/20addbf25b0676d7778efa6554023eb6f8ed4057)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Update documentation ([a9869f2](https://github.com/AndrejOrsula/pymoveit2/commit/a9869f2160efbe60bce6545d07d7a00de10db9af)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Bump package version in package.xml ([484272e](https://github.com/AndrejOrsula/pymoveit2/commit/484272e00bd1f132fde7a6efc90fc0f52a7f989a)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- [pre-commit.ci] pre-commit autoupdate ([6836e9a](https://github.com/AndrejOrsula/pymoveit2/commit/6836e9a5213fd2634754b627859ad0dffdbcb40c)) by [@pre-commit-ci[bot]](https://github.com/pre-commit-ci%5Bbot%5D) in [#12](https://github.com/AndrejOrsula/pymoveit2/pull/12)

## New Contributors

- [@egordon](https://github.com/egordon) made their first contribution in [#32](https://github.com/AndrejOrsula/pymoveit2/pull/32)
- [@amalnanavati](https://github.com/amalnanavati) made their first contribution in [#30](https://github.com/AndrejOrsula/pymoveit2/pull/30)
- [@pre-commit-ci[bot]](https://github.com/pre-commit-ci%5Bbot%5D) made their first contribution in [#22](https://github.com/AndrejOrsula/pymoveit2/pull/22)

## [2.1.0] - 2022-05-02

### Changed

- 2.1.0 ([8802000](https://github.com/AndrejOrsula/pymoveit2/commit/88020006d6361116d5809559232f608a37413c49)) by [@AndrejOrsula](https://github.com/AndrejOrsula) in [#11](https://github.com/AndrejOrsula/pymoveit2/pull/11)

## [2.0.0] - 2022-02-03

### Added

- Gripper_command: Add example ([67d9495](https://github.com/AndrejOrsula/pymoveit2/commit/67d9495a17cd9c300be924af4819782209f9978b)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Moveit2: Add support for multiple JointState publishers ([6cb4143](https://github.com/AndrejOrsula/pymoveit2/commit/6cb4143cf6ddedbff329a324d5833441b07594f5)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Moveit2_gripper: Add option to enable motion without planning ([1fb7ba8](https://github.com/AndrejOrsula/pymoveit2/commit/1fb7ba884e1643937a2426f5c1c6e3b7ce573b82)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Moveit2: Add reset option for `ignore_new_calls_while_executing` ([f41dbbe](https://github.com/AndrejOrsula/pymoveit2/commit/f41dbbef367d6ddf59f9967c587f6f9ec178efa2)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Moveit2: Add option for sync `reset_controller()` ([740563a](https://github.com/AndrejOrsula/pymoveit2/commit/740563a24cf77e911fcbe3ddee4df365c956576d)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Moveit2_servo: Add option for sync `enable()`/`disable()` ([f9de89c](https://github.com/AndrejOrsula/pymoveit2/commit/f9de89c3eb98ef9084e4a5d89f812966dc236490)) by [@AndrejOrsula](https://github.com/AndrejOrsula)

### Changed

- 2.0.0 ([70f8e00](https://github.com/AndrejOrsula/pymoveit2/commit/70f8e001b3de0167efb351e28ccfeec488534d5b)) by [@AndrejOrsula](https://github.com/AndrejOrsula) in [#3](https://github.com/AndrejOrsula/pymoveit2/pull/3)
- Gripper_command: Refactor and add improvements ([d4bceab](https://github.com/AndrejOrsula/pymoveit2/commit/d4bceabcdcc7df696db6abbb576397ee7de1c679)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Moveit2: Refactor `joint_state` assignment in requests ([2c69a16](https://github.com/AndrejOrsula/pymoveit2/commit/2c69a16d1bce89be193016993d87c3825a3412f2)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Moveit2: Do not initialise dummy joint states ([8b142a8](https://github.com/AndrejOrsula/pymoveit2/commit/8b142a85e6176b467bbd964f392234e747f26b96)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Moveit2: Refactor `compute_fk()` and `compute_ik()` ([03269f8](https://github.com/AndrejOrsula/pymoveit2/commit/03269f8b797c2a2487fbc3423c2be22eaa90a08b)) by [@AndrejOrsula](https://github.com/AndrejOrsula)

### Fixed

- Moveit2_gripper: Fix `is_open()` ([48c93c6](https://github.com/AndrejOrsula/pymoveit2/commit/48c93c6bec975f6510eb3888a631572f97722a1a)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Moveit2: Fix typo ([5cbb085](https://github.com/AndrejOrsula/pymoveit2/commit/5cbb08529a06d39d4544b392736058d499c989ca)) by [@AndrejOrsula](https://github.com/AndrejOrsula)

## [1.1.0] - 2022-01-03

### Added

- Add isort to pre-commit hooks ([4a29812](https://github.com/AndrejOrsula/pymoveit2/commit/4a298123f3b24bf6a48b14e3a2e8791e5204fb0e)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Moveit2_servo: Add option to enable Servo directly on command call ([48cdd87](https://github.com/AndrejOrsula/pymoveit2/commit/48cdd873a7c1501996efaa864ace27fc87d4594a)) by [@AndrejOrsula](https://github.com/AndrejOrsula)

### Changed

- 1.1.0 ([02478b4](https://github.com/AndrejOrsula/pymoveit2/commit/02478b4e10baebb726538b379f4663353c5344ee)) by [@AndrejOrsula](https://github.com/AndrejOrsula) in [#2](https://github.com/AndrejOrsula/pymoveit2/pull/2)
- Specify timeout when waiting for all service and action servers ([6b1ef36](https://github.com/AndrejOrsula/pymoveit2/commit/6b1ef36820cda4ac35c1b88a9f248b1862dfc79b)) by [@AndrejOrsula](https://github.com/AndrejOrsula)

### Fixed

- Fix typing of `wait_for_server_timeout_sec` arg ([8df89ad](https://github.com/AndrejOrsula/pymoveit2/commit/8df89ad6790fe59e104727ed66ee18851cb8f12e)) by [@AndrejOrsula](https://github.com/AndrejOrsula)
- Moveit2_gripper: Fix `reset_open()`/ `reset_closed()` ([f1836a7](https://github.com/AndrejOrsula/pymoveit2/commit/f1836a785f4fecb2d67e1608f80eb26b90f380d6)) by [@AndrejOrsula](https://github.com/AndrejOrsula)

## [1.0.0] - 2021-12-25

### Added

- Add initial version of pymoveit2 ([8870a88](https://github.com/AndrejOrsula/pymoveit2/commit/8870a884cb40e861749204af0dba997cc595c8ab)) by [@AndrejOrsula](https://github.com/AndrejOrsula) in [#1](https://github.com/AndrejOrsula/pymoveit2/pull/1)
- Add pre-commit configuration and setup ([7149b05](https://github.com/AndrejOrsula/pymoveit2/commit/7149b05b8a00a0d37b2ac19e95bd94619c15f9c4)) by [@AndrejOrsula](https://github.com/AndrejOrsula)

### Changed

- Initial commit ([dd2412d](https://github.com/AndrejOrsula/pymoveit2/commit/dd2412d6703e14d1d1f1d407ba4b1e3fbfa76933)) by [@AndrejOrsula](https://github.com/AndrejOrsula)

## New Contributors

- [@AndrejOrsula](https://github.com/AndrejOrsula) made their first contribution in [#1](https://github.com/AndrejOrsula/pymoveit2/pull/1)

<!-- generated by git-cliff -->

[1.1.0]: https://github.com/AndrejOrsula/pymoveit2/compare/1.0.0..1.1.0
[2.0.0]: https://github.com/AndrejOrsula/pymoveit2/compare/1.1.0..2.0.0
[2.1.0]: https://github.com/AndrejOrsula/pymoveit2/compare/2.0.0..2.1.0
[2.2.0]: https://github.com/AndrejOrsula/pymoveit2/compare/2.1.0..2.2.0
[3.0.0]: https://github.com/AndrejOrsula/pymoveit2/compare/2.2.0..3.0.0
[3.1.0]: https://github.com/AndrejOrsula/pymoveit2/compare/3.0.0..3.1.0
[3.2.0]: https://github.com/AndrejOrsula/pymoveit2/compare/3.1.0..3.2.0
[4.0.0]: https://github.com/AndrejOrsula/pymoveit2/compare/3.2.0..4.0.0
[4.1.0]: https://github.com/AndrejOrsula/pymoveit2/compare/4.0.0..4.1.0
[4.1.1]: https://github.com/AndrejOrsula/pymoveit2/compare/4.1.0..4.1.1
[4.2.0]: https://github.com/AndrejOrsula/pymoveit2/compare/4.1.1..4.2.0
[5.0.0]: https://github.com/AndrejOrsula/pymoveit2/compare/4.2.0..5.0.0
