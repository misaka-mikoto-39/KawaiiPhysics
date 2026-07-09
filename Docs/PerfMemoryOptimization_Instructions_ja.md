# KawaiiPhysics 高速化・メモリ削減 指示書

## Context(背景と制約)

KawaiiPhysics(v1.21.0、UE5.3〜5.7対応)のランタイムを調査した結果、**物理挙動を一切変えずに**適用できる最適化候補を特定した。本書は実装を担当するAIへの指示書である。

**絶対条件: 物理挙動を変えないこと。** 本編の全項目は Tier A(数学的に同一の演算をホイスト/キャッシュするだけ=ビット単位で同一結果)のみで構成する。浮動小数点の演算順序が変わるもの(Tier B)、挙動が変わりうるもの(Tier C)は本編から除外し、末尾に理由付きで列挙した。

対象: `Plugins/KawaiiPhysics/Source/KawaiiPhysics/`。以下、`Sim.cpp`=`Private/AnimNode_KawaiiPhysicsSimulation.cpp`、`Col.cpp`=`Private/AnimNode_KawaiiPhysicsCollision.cpp`、`Node.h`=`Public/AnimNode_KawaiiPhysics.h`。

**前提知識(重複実装の禁止)**: 本プラグインは既に多くの最適化を持つ。評価毎シミュレーション空間キャッシュ(`CurrentEvalSimSpaceCache`等)、各種スクラッチバッファ(`BridgeFeedback*`, `WorldCollisionHitsScratch`, SharedCollisionのswapバッファ)、風乱数のフレームキャッシュ、GameThreadでのSubsystemキャッシュ、固定サブステップ(`SimulateOnce`が最大4回/フレーム実行)、約40個のStatカウンター。これらを壊さない・重複させないこと。

ホットパスの構造: `EvaluateSkeletalControl_AnyThread` → `SimulateModifyBones` → `SimulateOnce`(×サブステップ数≤4) → ボーン毎に `Simulate()`/コリジョン/拘束。コストは **ボーン数 × サブステップ数 × (コライダー数+外力数)** でスケールする。

---

## 実装手順(優先度順・全Tier A)

**必ずStep 0を最初に行い、以降1ステップずつコミットして都度検証すること。**

### Step 0 — ゴールデン位置回帰テストを先に追加(変更前の値で固定)
- `Private/Tests/KawaiiPhysicsSimulationTest.cpp` に新テスト `KawaiiPhysics.Simulation.GoldenPositions` を追加。
- `FKawaiiPhysicsTestAccessor`(`Private/Tests/KawaiiPhysicsTestHarness.h`)を使い、①固定サブステップの縦チェーン ②BoneConstraint付き2本チェーン ③カプセル/ボックス/平面コライダー付きチェーン、を約200フレーム実行し、全ボーンの `Location` を**変更前ビルドで取得したリテラル値**と**完全一致**(`==`。Tier Aはビット一致が約束できるため)で比較する。
- **リテラル値の採取は一切の最適化を入れる前に行うこと。** 以降の各Stepでこのテストが1つでも差分を出したら、そのStepはTier A違反 → 差し戻して再設計。

### Step 1 — シミュレーション空間キャッシュの値返しを参照返しに (`Sim.cpp:880-901`, `Sim.cpp:871-878`, `Node.h:1339-1341`)
- `GetSimulationSpaceCacheFor` は `FSimulationSpaceCache`(FTransform×2 ≒192B)を**値返し**しており、変換1回につき2回呼ばれる。変換はボーン毎(WorldSpace/BaseBoneSpaceシム時、WorldCollision、`UpdateModifyBonesPoseTransform`)・コライダー毎に発生。
- シグネチャを `const FSimulationSpaceCache&` に変更。
  - ComponentSpace → 関数ローカル `static const FSimulationSpaceCache IdentityCache;` を返す(マジックスタティックでスレッド安全・不変)。
  - メンバキャッシュ命中時 → `CurrentEvalSimSpaceCache` / `CurrentEvalWorldSpaceCache` を参照返し。
  - フォールバック(SimulationSpace≠BaseBoneSpace時のBaseBoneSpace要求のみ) → `mutable FSimulationSpaceCache FallbackSimSpaceCacheScratch;` をNode.hに追加して書き込み・返却。**1回の変換で2つの引数が両方フォールバックになることはない**(空間は3種で From==To は早期return済み)— この不変条件をコメント+dev版`ensure`で明記。
- 効果: 大。全空間変換からコピー2個を除去。

### Step 2 — Stiffness の Pow をメモ化 (`Sim.cpp:713-720`, `SimulateOnce` 呼び出し側 `Sim.cpp:280-301`)
- `ApplyStiffnessPull` は `1 - FMath::Pow(1-Stiffness, Exponent)` をボーン毎×サブステップ毎に計算。powfはホットパス最重量級。
- **注意: `Exponent==1` (サブステップ時) の `Pow(x,1)==x` ショートカットは実装しないこと**(`(1.0f/N)*N` が正確に1.0fになる保証も、`powf(x,1.0f)==x` のビット一致保証もない=Tier B)。
- 代わりに入力キーのメモ化: Nodeに `struct FStiffnessMemo { float Stiffness; float Exponent; float PullAlpha; }` の `TArray`(サイズ=ModifyBones.Num、不一致時SetNum)を追加。ボーン毎に `Stiffness != Memo.Stiffness || Exponent != Memo.Exponent`(**厳密なfloat比較**)なら同一式・同一`FMath::Pow`で再計算して格納、一致ならキャッシュ値を使用。同じ関数に同じ入力→同じビット、入力変化は毎ステップ検知するので発散しない(外力がBone.PhysicsSettingsを書き換えるケースも安全)。
- 効果: 大。設定が安定していれば初回以降Powがほぼゼロに(4サブステップ時は最大3/4削減保証)。

### Step 3 — カーブ評価のホイスト+全カーブ空の高速パス (`Sim.cpp:36-74` `UpdatePhysicsSettingsOfModifyBones`)
- 現状: `GetRichCurveConst()`×6 + `Eval`×6 をボーン毎・毎フレーム実行(デフォルト `bUpdatePhysicsSettingsInGame=true` で全員が通る)。
- ①6本の `GetRichCurveConst()` ポインタをループ外へホイスト(無条件でTier A)。
- ②毎フレーム、6本すべて `IsEmpty()` かチェック。全て空なら: 空カーブの `Eval(x, 1.0f)` は正確に `1.0f` を返し `X * 1.0f == X` はビット一致なので、6個の Clamp/Max 済み値を**1回だけ**計算して全ボーンに代入。空でないカーブが1本でもあれば従来通り(ホイスト済みポインタ使用)。空チェックは毎フレーム行う(カーブもピン公開可能なため)。
- 効果: 大(カーブ未使用=既定構成の全ユーザーに効く)。

### Step 4 — コリジョン形状のステップ単位キャッシュ (`Col.cpp:516-624`, `Sim.cpp:388-425`, `Public/KawaiiPhysicsCollisionLimits.h`, `Tests/KawaiiPhysicsTestHarness.h:300-320,505-520`)
- **平面(ストレージ不要)**: `Col.cpp:621` の `Planar.Rotation.GetUpVector()` を `FVector(Planar.Plane.X, Planar.Plane.Y, Planar.Plane.Z)` に置換。全ての `Plane` 構築箇所(`Col.cpp:290,306,1040,1104`)が同じ `Rotation.GetUpVector()` から `FPlane` を作っており法線はビット一致(検証済み)。テストフィクスチャ(`KawaiiPhysicsCollisionTest.cpp:181`)も整合。
- **カプセル**: `FCapsuleLimit` に非UPROPERTYフィールド `FVector CachedStartPoint/CachedEndPoint` と `void UpdateRuntimeCache()`(`Col.cpp:525-526` と**同一式**)を追加。
- **ボックス**: `FBoxLimit` に `FTransform CachedBoxTransform`(必要なら `FBox CachedLocalBox` も)+ `UpdateRuntimeCache()`。
- Node側に `PrepareCollisionShapeCaches()` を追加し、`SimulateOnce` のコリジョン節冒頭(`Sim.cpp:392`付近)で `CapsuleLimits/CapsuleLimitsData/SharedCapsuleLimits/BoxLimits/BoxLimitsData/SharedBoxLimits` の有効な各limitの `UpdateRuntimeCache()` を毎ステップ呼ぶ。
  - **毎ステップ再計算にする理由**: `FCollisionLimitBase` 系は**新フィールドをコピーしない自前 `operator=`** を持ち(`KawaiiPhysicsCollisionLimits.h:87-101`)、SharedCollisionはlimitをコピー後にLocation/Rotationを上書きする。代入経由のキャッシュは陳腐化リスクがあるため、構造的に陳腐化不可能な毎ステップ再計算(コスト=形状数×≤4回/フレーム、従来の1/ボーン数)にする。
- **テストハーネス修正必須**: `StepOnce`(`KawaiiPhysicsTestHarness.h:512`付近)と直接呼び出しヘルパー(`:306-314`)は `AdjustByCapsuleCollision`/`AdjustByBoxCollision` の前に prepare を呼ぶこと。
- 効果: 中〜大(ボーン数×形状数×サブステップでスケール。カプセル複数のスカート/髪で顕著)。

### Step 5 — 外力解決のボーンループ外ホイスト (`Sim.cpp:550-645`, `Sim.cpp:263-302`)
- 現状: `ExternalForces[i].IsValid()` + `GetMutablePtr<T>`(IsChildOf走査)と `CustomExternalForces[i]` のnullチェックが**ボーン毎×外力毎**。
- `SimulateOnce` 冒頭で `TArray<FKawaiiPhysics_ExternalForce*>`(メンバスクラッチ、`Reset()`)と `TArray<UKawaiiPhysics_CustomExternalForce*>` を1回構築し、`Simulate()` はそれを走査。**ボーン毎の `bIsEnabled` チェックは残す**(ループ中のトグル反映を厳密に維持)。
- `ResolveExternalForceBoneTransform` は不変のポーズしか読まないため、BoneSpace外力が複数あっても**ボーン毎に1回だけ**遅延計算に変更(現在はBoneSpace外力毎に再計算)。
- 元コードのコメントが警告する「Apply中の配列変更」は既存コードではPostApply(ループ後)のみ。dev版 `ensure(GetData()/Num()不変)` をSimulateループ後に追加し、Apply中の変更は非サポートと明記。
- 効果: 中(外力を使うノードのみだが、そこでは外力数×ボーン数×サブステップの走査を除去)。

### Step 6 — WorldCollision クエリパラメータのホイスト (`Col.cpp:317-352`, 呼び出し側 `Sim.cpp:392-426`)
- 現状: `FCollisionQueryParams`(`AddIgnoredComponent`=ヒープ確保)、`TraceChannel`、`FCollisionResponseParams`(レスポンスコンテナのコピー)、`GetCollisionObjectType()/GetCollisionResponseToChannels()` の UObject 読みを**ボーン毎×サブステップ毎**に実行。
- これらはノード設定と `OwningComp` にのみ依存 → `bAllowWorldCollision` 時のみ `SimulateOnce` のボーンループ前に1回構築し、const参照渡し(またはステップ単位スクラッチメンバ)。ボーン毎の早期return(ParentIndex/半径)は現状維持。
- 効果: WorldCollision使用者には中(sweep自体は残るがパラメータ構築のゴミを全廃)。未使用者にはゼロ。

### Step 7 — XPBDコンプライアンス表のステップ単位事前計算 (`Col.cpp:690-738` `AdjustByBoneConstraints`)
- `Compliance /= StepDt*StepDt` と `FMath::Max(GetStepDeltaTime(), KINDA_SMALL_NUMBER)` が制約毎×反復毎に実行されている。
- 関数冒頭(または `SimulateOnce` でステップ毎に1回)で `StepDt` を確定し、7要素の `float CompliancePerDt2[7]` を**同一式** `XPBDComplianceValues[i] / (StepDt * StepDt)` で構築。制約毎はindexクランプ(現状維持)+表引きのみに。
- 効果: 小〜中(布グリッドの制約多数×反復×サブステップ時)。

### Step 8 — 小粒の確実な改善
- `ApplySimulateResult`(`Sim.cpp:797-811`): 最初のループ前に `OutBoneTransforms.Reserve(ModifyBones.Num())`(ループはちょうどその数をAddするので正確)。
- `ExternalForce_Curve::PreApply`(`ExternalForces/KawaiiPhysicsExternalForce_Curve.cpp:55-101`): Average/Max/Minモードの毎フレーム `TArray<FVector> CurveValues` ヒープ確保を廃止し、**同じ加算/fold順序**のインライン累積に置換(Averageは走査和→最後に除算、Max/Minは同順のfold)。

### Step 9 — メモリ/シリアライズ削減
- `Public/ExternalForces/KawaiiPhysicsExternalForce.h:110-121`: `RandomizedForceScale` / `Force` / `ComponentTransform` を `UPROPERTY(Transient)` に(毎フレームPreApplyで読み出し前に再計算されることを検証済み。Transient化はセーブ互換)。軽微な副作用: 初回評価前のエディタデバッグ描画で古い保存値ではなくゼロ表示になるだけ。
- `Public/KawaiiPhysicsBoneConstraintTypes.h:56-74`: `ModifyBoneIndex1/2`, `Length`, `bIsDummy`, `Lambda` を `UPROPERTY(Transient)` に(`InitBoneConstraints` が `MergedBoneConstraints` 上で毎回再計算することを検証済み)。
- `Public/KawaiiPhysicsTypes.h:135-261`: `FKawaiiPhysicsModifyBone` のbool 4個(`bDummy/bInterBoneDummy/bBridgeDummy/bSkipSimulate`)をint32/floatブロックに隣接するよう並べ替えてパディング除去(約8〜16B/ボーン。タグ付きシリアライズは順序非依存)。
- **`FModifyBoneConstraintData::BoneName1/2` は絶対に触らないこと**: 一見deprecatedだが `UKawaiiPhysicsBoneConstraintsDataAsset::PostLoad`(`KawaiiPhysicsBoneConstraintsDataAsset.cpp:77-95`)のレガシー移行の**入力元**。Transient/WITH_EDITORONLY_DATA化すると旧アセットの読み込みが静かに壊れる。
- 効果: ABPアセットサイズ+ランタイムメモリ(約8〜16B/ボーン)。

### Step 10 — 開発ビルドのStatスコープ整理+初期化の微最適化
- ボーン毎に構築/破棄されるスコープカウンターをループ外へ: `STAT_KawaiiPhysics_Simulate`(`Sim.cpp:555`→SimulateループをSimulateOnce側で包む)、`STAT_KawaiiPhysics_AdjustByCollision`(`Sim.cpp:400`)、`STAT_KawaiiPhysics_UpdateModifyBonesPoseTransform`(`AnimNode_KawaiiPhysicsModifyBones.cpp:525`付近)。変換系(`ConvertSimulationSpace*`)と風のカウンターは変換コストの唯一の可視化手段なので**現状維持**(作者判断事項)。stat UIの「呼び出し回数」の意味が per-bone → per-loop に変わる点をコミットメッセージに明記。
- `InitBoneConstraints`(`Col.cpp:740-770`): `IndexOfByPredicate` O(N)×制約数 → `TMap<FName,int32>`(BoneName→index)を1回構築して引く。述語ラムダの `[Constraint]`(構造体まるごと値キャプチャ)を参照キャプチャに。初期化時のみだが無料で直せる。

---

## 実装しないこと(重要・理由付き)

| 項目 | 理由 |
|---|---|
| `Pow(x, 1.0f) == x` ショートカット | ビット一致保証なし(Tier B)。Step 2のメモ化で実質不要 |
| `ApplySimulateResult` の `GetSafeNormal()==GetSafeNormal()` 判定の置き換え | より安価な述語は早期returnするボーンの集合を変える=挙動変化リスク。現状維持 |
| 風サンプリングのボーンループ外ホイスト | 風は各ボーンのポーズ位置でサンプルする仕様。ホイストは結果が変わる |
| 風キャッシュ配列の `SetNumZeroed` 削減 | 容量再利用済みで効果が無視できる |
| `OffsetRotation.Quaternion()` のキャッシュ | limit毎に毎フレーム1回だけ。ピン編集可能なRotatorのダーティ追跡が必要でリスク>効果 |
| `FKawaiiPhysicsModifyBone` のhot/cold分離・SoA化 | Tier C。BP公開構造体の大改修で回帰リスク大。将来の任意課題 |
| エディタの毎フレーム `Apply*DataAsset` への変更検知 | Shippingに影響ゼロ(エディタUXのみ)。`PostEditChangeProperty/PostEditUndo/PostTransacted` での変更シリアル実装が必要で、検知漏れ=ライブ編集破壊。やるなら別PRでオプトイン |
| コリジョンループの形状外側化 | 作者がキャッシュ効率のためボーン外側を明示選択済み(`Sim.cpp:388-391` コメント)。尊重する |

---

## 検証手順

このリポジトリにCIは無く、コンテナ環境ではUEをコンパイルできない。**作者マシン(UE5.3〜5.7、リポジトリ直下のサンプルプロジェクト)での検証を前提とする。**

1. **変更前にベースライン取得**:
   `UnrealEditor-Cmd <path>\KawaiiPhysicsSample.uproject -ExecCmds="Automation RunTests KawaiiPhysics; Quit" -unattended -nopause -nullrhi -log`
   (既存スイート: `KawaiiPhysics.Simulation.Determinism / IntegrationCore / BoneConstraintStepDeltaTime / ParameterResponse / FramerateIndependence / NumericalStability / SyncBone*`、`KawaiiPhysics.Collision.*`)
2. **Step 0のゴールデンテストを最初に追加**し、変更前ビルドでリテラル値を採取してコミット。
3. Step 1〜10 を**1つずつ**実装・コミットし、都度スイート+ゴールデンテストを実行。ゴールデン値に差分が出たらそのStepはTier A違反として差し戻し。
4. 自動テストが無い経路の手動確認: WorldCollision(Step 6、サンプルマップ)/ LimitsDataAsset・BoneConstraintsDataAssetのライブ編集(Step 9)/ 旧形式BoneConstraintsアセットの移行ログ(`Update : BoneName -> BoneReference` が出ること)。
5. **性能計測**: `stat anim` + `STAT_KawaiiPhysics_*`。多ボーンのスカート構成、4サブステップ、WorldSpaceシム、カプセル+BoneConstraint有効で before/after を比較。Step 10でカウンター粒度が変わるため、計測はStep 10より前に行うか `STAT_KawaiiPhysics_Eval` 合計で比較。
6. 完了後、ブランチ `claude/perf-memory-optimization-nwlw2u` にpush。

## 期待効果(定性)

- 全ユーザー: Step 1(空間変換コピー除去)+ Step 2(Pow削減)+ Step 3(カーブ評価)で per-bone × per-substep の固定費を大きく削減。
- コライダー使用時: Step 4 で形状数×ボーン数分の再計算を形状数分に。
- WorldCollision/外力/BoneConstraint使用時: Step 5〜7 でそれぞれの per-bone ゴミを除去。
- メモリ: Step 9 でボーン毎約8〜16B+アセットのシリアライズ削減。挙動はすべてビット単位で不変。
