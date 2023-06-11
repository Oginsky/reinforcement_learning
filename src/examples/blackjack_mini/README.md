# Карточная игра "Blackjack"
Применение методов `Monte Carlo` и `Sarsa(λ)` на примере карточной игры "Blackjack" с упрощенными правилами.

## Правила

+ Игра играется с бесконечной колодой (т.е. карты выбираются «с возвратом»)
+ Каждая карта, выбираемая из колоды, имеет стоимость от 1 до 10 (вероятность вытащить
любое значение – 1/10), а также либо красный цвет (с вероятностью 1/3), либо черный (с
вероятностью 2/3)
+ В колоде нет тузов и карт с персонажами
+ В начале игры сдающий и игрок достают по одной черной карте и кладут ее лицом вверх
+ Во время каждого хода игрок может выполнить одно из двух действий, которые мы будем
обозначать кодовыми названиями «еще» и «хватит»
+ Если игрок выполняет действие «еще», то он берет одну карту из колоды
+ Если игрок выполняет действие «хватит», то ход переходит сдающему
+ Сумма очков вычисляется следующим образом: значения черных карт прибавляются, а
красных – вычитаются из общей суммы
+ Если сумма очков превышает 21 или становится отрицательной, то игрок проигрывает
(получает награду -1) и игра заканчивается
+ Сдающий всегда набирает карты, пока не получит сумму очков, равную или превосходящую 17. Если сдающий набирает больше 21 или меньше 0 очков, то он проигрывает, а игрок
выигрывает (получает награду +1). В противном случае победа определяется по сумме
очков: у кого больше, тот и выиграл. Ничья приносит нулевую награду

## Результаты

### `Monte Carlo` (500 000 эпизодов)

+ График функции состояния:
![state function graph](https://github.com/Oginsky/reinforcement_learning/raw/main/data/graphs/blackjack_mini/value_function_500k_episodes.png)
---

### `Sarsa(λ)` (10 000 эпизодов)

<details>
<summary>Кривая обучения (λ = 0.0)</summary>
<img alt="learning curve graph" src="https://github.com/Oginsky/reinforcement_learning/raw/main/data/graphs/blackjack_mini/mse_episode_lambda=0_10000.png" alt=""></img>
</details>

<details>
<summary>Кривая обучения (λ = 1.0)</summary>
<img alt="learning curve graph" src="https://github.com/Oginsky/reinforcement_learning/raw/main/data/graphs/blackjack_mini/mse_episode_lambda=1_10000.png" alt=""></img>
</details>

<details>
<summary>Зависимость среднеквадратической ошибки от λ</summary>
<img alt="learning curve graph" src="https://github.com/Oginsky/reinforcement_learning/raw/main/data/graphs/blackjack_mini/mse_on_lambda_10000e.png" alt=""></img>
</details>

---


## Линейная аппроксимация на основе грубого кодирования (coarse coding)

> В этом разделе приводятся результаты аппроксимации состояния агента. 
> Используется бинарный вектор признаков 𝜙(𝑠, 𝑎) с 3 ∗ 6 ∗ 2 = 36 признаками. Каждый
> бинарный признак имеет значение 1, если (𝑠, 𝑎) лежит в кубоиде состояния, соответствующего
> этому признаку, и действия, соответствующему этому признаку. Кубоиды имеют следующие
> пересекающиеся интервалы:

> 𝑑𝑒𝑎𝑙𝑒𝑟(𝑠) = {[1,4],[4,7],[7,10]} <br>
> 𝑝𝑙𝑎𝑦𝑒𝑟(𝑠) = {[1,6],[4,9],[7,12],[10,15],[13,18],[16,21]} <br>
> 𝑎 = {ℎ𝑖𝑡, 𝑠𝑡𝑖𝑐𝑘} <br>

> функции ценности аппроксимируется: <br>
> 𝑄(𝑠, 𝑎) = 𝜙(𝑠, 𝑎)𝑇θ

### `Monte Carlo` (100 000 эпизодов)

+ График функции состояния:
![state function graph](https://github.com/Oginsky/reinforcement_learning/raw/main/data/graphs/blackjack_mini/linear_value_function_100k_episodes.png)
---

### `Sarsa(λ)` (10 000 эпизодов)

<details>
<summary>Кривая обучения (λ = 0.0)</summary>
<img alt="learning curve graph" src="https://github.com/Oginsky/reinforcement_learning/raw/main/data/graphs/blackjack_mini/linear_mse_episode_lambda=0_10000.png" alt=""></img>
</details>

<details>
<summary>Кривая обучения (λ = 1.0)</summary>
<img alt="learning curve graph" src="https://github.com/Oginsky/reinforcement_learning/raw/main/data/graphs/blackjack_mini/linear_mse_episode_lambda=1_10000.png" alt=""></img>
</details>

<details>
<summary>Зависимость среднеквадратической ошибки от λ</summary>
<img alt="learning curve graph" src="https://github.com/Oginsky/reinforcement_learning/raw/main/data/graphs/blackjack_mini/linear_mse_on_lambda_10000e.png" alt=""></img>
</details>