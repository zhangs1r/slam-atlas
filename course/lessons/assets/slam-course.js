window.lessonKit = {
  answer(button, targetId) {
    const target = document.getElementById(targetId);
    if (!target) return;
    const correct = button.dataset.correct === "yes";
    const explanation = button.dataset.explanation || "";
    target.className = "quiz-feedback show " + (correct ? "correct" : "wrong");
    target.innerHTML = (correct ? "<strong>回答到位：</strong>" : "<strong>再想一步：</strong>") + explanation;
  },
  openTab(group, id) {
    document.querySelectorAll(`[data-tab-panel="${group}"]`).forEach((panel) => {
      panel.classList.toggle("active", panel.dataset.tabId === id);
    });
    document.querySelectorAll(`[data-tab-button="${group}"]`).forEach((button) => {
      const active = button.dataset.tabId === id;
      button.style.background = active ? "#cc785c" : "#fff7f3";
      button.style.color = active ? "#ffffff" : "#7d4c3b";
    });
  }
};
document.addEventListener("DOMContentLoaded", () => {
  document.querySelectorAll(".tab-group").forEach((group) => {
    const first = group.querySelector("[data-tab-button]");
    if (first) {
      window.lessonKit.openTab(first.dataset.tabButton, first.dataset.tabId);
    }
  });
});
