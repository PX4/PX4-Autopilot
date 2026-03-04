# 번역

We'd love your help to translate _QGroundControl_, PX4 Metadata (in QGC), and our guides for PX4, _QGroundControl_ and MAVLink!

Our docs (and _QGroundControl_) use the [Crowdin](https://crowdin.com) online tool for translation.
Crowdin은 Github에서 원본 문서를 가져와서 번역하고 검토(승인)할 문서들을  편집할 수 있습니다.

Crowdin은 "풀 요청"(이 단계에서 개발팀이 주기적으로 검토하고 승인해줍니다) 방식으로 Github에 번역한 문서를 내보냅니다.
내보낸 출력물에는 번역한 원본 문서와 번역한 문자열로 바뀌어 승인한 텍스트가 들어있습니다(예: 문자열을 번역하지 않았거나 바꾸지 않았다면, 영문으로 그대로 나타냅니다).

:::tip
You will need a (free) [Crowdin account](https://crowdin.com/join) account to join the translation team!
:::

:::info
The benefit of this system is that the translation closely tracks the source documents.
독자 입장에서는 오래되어 때가 지난 번역을 보고 오해할 일이 없습니다.
:::

## 시작하기

번역 팀에 참여하는 방법은 다음과 같습니다:

1. Join Crowdin: [https://crowdin.com/join](https://crowdin.com/join)

2. 참여를 원하는 번역 프로젝트를 엽니다:
   - [QGroundControl](https://crowdin.com/project/qgroundcontrol) — QGroundControl UI and hard coded strings.
   - [PX4-Metadata-Translations](https://crowdin.com/project/px4-metadata-translations) — PX4 parameter and event descriptions in QGroundControl.
   - [PX4 User Guide](https://crowdin.com/project/px4-user-guide)
   - [QGroundControl Developer Guide](https://crowdin.com/project/qgroundcontrol-developer-guide)
   - [QGroundControl User Guide](https://crowdin.com/project/qgroundcontrol-user-guide)
   - [MAVLink Guide](https://crowdin.com/project/mavlink)

3. 번역하려는 언어를 선택합니다

4. Click the **Join** button (next to the text _You must join the translators team to be able to participate in this project_)

   ::: info
   You will be notified once your application to join is accepted.

:::

5. 번역을 시작하세요!

## 별도 참고

### 접두문을 수정하지 마십시오.

Vuepress uses `:::` to mark the beginning of notes, tips and warning:

```html
:::tip
The text for the tip.
:::
```

The text for `:::tip` or `:::warning` etc. should not be modified as it defines the colour of the notebox.

## 새로운 언어 추가

번역 대상 언어가 없으면, 프로젝트 관리자에게 요청하십시오.(각 프로젝트 홈페이지에 연락처 링크가 있음).

:::warning
번역 작업은 어렵습니다.
새로운 언어 번역을 요청하기 전에, 번역을 도와줄 다른 사람이 있는지 찾아보십시오.
:::

## 도움 받기

The _Crowdin_ interface is self explanatory, but there is plenty of additional information on the [knowledgeable](https://support.crowdin.com/).

You can also ask for help from translators and developers in the Dronecode community using [our support channels](../contribute/support.md).
