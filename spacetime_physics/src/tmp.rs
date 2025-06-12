use spacetimedb::{
    client_visibility_filter, reducer, table, Filter, Identity, ReducerContext, Table,
};

#[reducer(client_connected)]
fn on_connect(ctx: &ReducerContext) {
    ctx.db.account().insert(Account {
        identity: ctx.sender,
        name: ctx.sender.to_abbreviated_hex().to_string(),
        online: true,
    });
}

#[table(name = account, public)]
pub struct Account {
    #[primary_key]
    pub identity: Identity,
    #[unique]
    #[index(btree)]
    pub name: String,
    pub online: bool,
}

#[client_visibility_filter]
const ACCOUNT_FILTER: Filter =
    Filter::Sql("SELECT * FROM account WHERE account.identity = :sender");
